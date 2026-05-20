import cv2
import time
import threading
import numpy as np
from models.cam import Camera
from models.detector import Detector
from models.tracker import Tracker, Status
from models.stepper import SysParams, EmmMotor
from models.pid import PIDController
import Hobot.GPIO as GPIO
from models.status import GPIN
from models.dm_imu import imu


# --------- 放在最前面：特别注意的接口和开关 ---------------------
camera_index = 0             # 摄像头索引，需根据实际情况调整
yaw_port = '/dev/ttyS1'      # yaw轴电机串口
pitch_port = '/dev/ttyS2'    # pitch轴电机串口

use_kf = True           # 是否启用卡尔曼滤波
show_windows = True     # 是否显示调试窗口
# ---------------------------------------------------------

# ----------------- 模块初始化 --------------------------------------------------------------
camera = Camera(index = camera_index, width = 640, height = 480)
detector = Detector(min_area = 3000, max_area = 500000)
tracker = Tracker(f_pixel_h = 725.6, real_height = 17.5, use_kf = use_kf) 

stepper_yaw = EmmMotor(port = yaw_port, baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = EmmMotor(port = pitch_port, baudrate = 115200, timeout = 1, motor_id = 2)
pid_yaw = PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)

lazer = GPIN(pin=16, mode=1)        # 激光笔控制
heart_beat = GPIN(pin=18, mode=1)   # 呼吸灯，用于表示主程序还在跑
# ----------------------------------------------------------------------------------------

# ------------------ 全局变量 -----------------------
# 世界坐标系下的目标位置，单位为度
system_running = True    # 控制线程状态
w_target_yaw = 0.0
w_target_pitch = 0.0
# 供控制线程读取的滑块参数
ctrl_vel_rpm = 3000
ctrl_acc = 0

# ======= 新增：全自主盲搜状态量 =======
has_tracked_ever = False     # 记录自上电起，是否至少成功捕获过一次目标
yaw_scan_speed = 3.0         # 盲搜旋转速度 (度/秒)，可通过滑块调节
# ====================================

# ------------------------------------------------

def nothing(x):
    pass

def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 300, 150)
    cv2.namedWindow('DETECTOR', cv2.WINDOW_FREERATIO)  
    cv2.namedWindow('BIN', cv2.WINDOW_FREERATIO)      

    cv2.createTrackbar('yaw_kp', 'Controls', 6, 100, nothing)   
    cv2.createTrackbar('yaw_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('yaw_kd', 'Controls', 1, 100, nothing)

    cv2.createTrackbar('pitch_kp', 'Controls', 5, 100, nothing)   
    cv2.createTrackbar('pitch_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('pitch_kd', 'Controls', 0, 100, nothing)  

    cv2.createTrackbar('onfire_tol', 'Controls', 8, 100, nothing) # 开火容忍度，单位为0.1度

    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)

    # ======= 新增：盲搜巡航速度滑块 =======
    cv2.createTrackbar('scan_speed', 'Controls', 3, 50, nothing) # 默认3度/秒
    # ====================================

    # 视差参数 (范围 -5.00cm 到 +5.00cm)   单位：0.01cm
    cv2.createTrackbar('ref_x', 'Controls', 475, 1000, nothing)    # 初始值 485 -> -0.15
    cv2.createTrackbar('ref_y', 'Controls', 610, 1000, nothing)    # 初始值 635 -> 1.35
    cv2.createTrackbar('ref_z', 'Controls', 500, 1000, nothing)    # 初始值 500 -> 0.0

    # 角度偏差补偿 (范围 -5.0度 到 +5.0度)     单位: 0.1度
    cv2.createTrackbar('yaw_bias', 'Controls', 30, 100, nothing)   # 初始值 27 -> -2.3
    cv2.createTrackbar('pitch_bias', 'Controls', 49, 100, nothing) # 初始值 48 -> -0.2

def update_params():
    """回调获取滑块参数"""
    global yaw_scan_speed

    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')/1000
    yaw_ki = cv2.getTrackbarPos('yaw_ki', 'Controls')/1000
    yaw_kd = cv2.getTrackbarPos('yaw_kd', 'Controls')/10000

    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls')/1000
    pitch_ki = cv2.getTrackbarPos('pitch_ki', 'Controls')/1000
    pitch_kd = cv2.getTrackbarPos('pitch_kd', 'Controls')/10000

    onfire_tol = cv2.getTrackbarPos('onfire_tol', 'Controls')/10

    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')
    show_val = cv2.getTrackbarPos('show', 'Controls')

    # 获取盲搜速度
    yaw_scan_speed = float(cv2.getTrackbarPos('scan_speed', 'Controls'))

    # 视差与角度偏差，映射回真实的小数和负数
    ref_x = (cv2.getTrackbarPos('ref_x', 'Controls') - 500) / 100.0
    ref_y = (cv2.getTrackbarPos('ref_y', 'Controls') - 500) / 100.0
    ref_z = (cv2.getTrackbarPos('ref_z', 'Controls') - 500) / 100.0
    yaw_bias = (cv2.getTrackbarPos('yaw_bias', 'Controls') - 50) / 10.0
    pitch_bias = (cv2.getTrackbarPos('pitch_bias', 'Controls') - 50) / 10.0

    # 赋值给 tracker 模块
    tracker.onfire_tol = onfire_tol
    tracker.ref_point = np.array([ref_x, ref_y, ref_z])
    tracker.yaw_bias = yaw_bias
    tracker.pitch_bias = pitch_bias

    #赋值给 pid 模块
    pid_yaw.set_Kp(yaw_kp)
    pid_yaw.set_Ki(yaw_ki)
    pid_yaw.set_Kd(yaw_kd)
    
    pid_pitch.set_Kp(pitch_kp)
    pid_pitch.set_Ki(pitch_ki)
    pid_pitch.set_Kd(pitch_kd)

    return vel_rpm, acc, show_val

def control_thread():
    """
    内环：高频姿态控制线程
    以 200Hz 运行，死咬靶纸的世界坐标，疯狂下发指令对抗底盘运动
    """
    global w_target_yaw, w_target_pitch, system_running, ctrl_vel_rpm, ctrl_acc

    while system_running:
        start_time = time.time()
        
        # 极速读取 IMU
        pkt, _, _ = imu.dev.get_latest()
        curr_yaw, curr_pitch = 0.0, 0.0
        if pkt is not None:
            _, (_, _, curr_yaw) = pkt
            
        # 计算空间角度误差
        error_yaw = w_target_yaw - curr_yaw
        error_pitch = w_target_pitch - curr_pitch

        # 将绝对角度误差归一化到 -180 ~ 180，防止临界点云台暴冲乱转
        error_yaw = (error_yaw + 180) % 360 - 180

        # PID 计算出目标速度
        correction_yaw = pid_yaw.compute(error_yaw)
        correction_pitch = pid_pitch.compute(error_pitch)
        
        # 映射为电机的绝对转速 (RPM)
        # 当滑块里的 kp 依然是 /1000 的小数值，这里必须乘以 1000 放大，否则电机没劲
        # 但如果发现 /1000调起来灵敏度太高，换成 /10000后此处映射也无需改动，否则白改了
        vel_out_yaw = int(abs(correction_yaw) * 1000)
        vel_out_pitch = int(abs(correction_pitch) * 1000)

        # 速度封顶保护
        vel_out_yaw = min(vel_out_yaw, ctrl_vel_rpm)
        vel_out_pitch = min(vel_out_pitch, ctrl_vel_rpm)

        # 判断转向逻辑，如果正反馈直接把这里的 0 和 1 互换！
        dir_yaw = 1 if correction_yaw > 0 else 0
        dir_pitch = 1 if correction_pitch > 0 else 0
        
        # 下发纯速度指令
        try:
            # ============= 新增：调试打印 ================
            # 每隔一段时间打印一次（防止刷屏太快看不清终端）
            # q指令 -> Error:{error_yaw:.2f}度 | Kp:{pid_yaw.Kp:.4f} | 下发速度:{vel_out_yaw} RPM")
            # ============================================
            
            # 加入 0.05 度的死区，防止到位后电机高频微调发出滋滋声
            if abs(error_yaw) > 0.05:
                stepper_yaw.emm_v5_vel_control(dir=dir_yaw, vel=vel_out_yaw, acc=ctrl_acc, snF=False)
            else:
                stepper_yaw.emm_v5_vel_control(dir=dir_yaw, vel=0, acc=ctrl_acc, snF=False) # 瞬间刹停
                
            if abs(error_pitch) > 0.05:
                stepper_pitch.emm_v5_vel_control(dir=dir_pitch, vel=vel_out_pitch, acc=ctrl_acc, snF=False)
            else:
                stepper_pitch.emm_v5_vel_control(dir=dir_pitch, vel=0, acc=ctrl_acc, snF=False)
        except Exception as e:
            print(f"电机通讯失败: {e}")
            
        # 严格保持 200Hz 刷新率
        cost_time = time.time() - start_time
        sleep_time = 0.005 - cost_time
        if sleep_time > 0:
            time.sleep(sleep_time)
            
def main():
    global w_target_yaw, w_target_pitch, system_running, ctrl_vel_rpm, ctrl_acc, show_windows
    global has_tracked_ever, yaw_scan_speed

    print("视觉跟踪系统启动... 按 'q' 键退出。")
    init_board()
    prev_time = time.time()

    # 启动高速控制线程
    ctrl_thread = threading.Thread(target=control_thread, daemon=True)
    ctrl_thread.start()
    print("高速 IMU 控制内环已启动")

    try:
        while True:

            # 呼吸灯，证明主程序在运行
            heart_beat.flash()

            # 读帧
            ret, frame = camera.read()
            if not ret: continue

            # 更新参数
            ctrl_vel_rpm, ctrl_acc, show_windows= update_params()

            # 目标检测
            target = detector.detect(frame)
            
            # 滤波跟踪与解算
            vis_yaw, vis_pitch, dist, status, laser_pos = tracker.track(target)
            
            # 计算 FPS（终端打印）
            curr_time = time.time()
            loop_dt = max(curr_time - prev_time, 1e-6)
            fps = 1.0 / loop_dt
            prev_time = curr_time

            # 坐标系变换与目标刷新
            if status in [Status.TRACK, Status.TMP_LOST]:
                has_tracked_ever = True         # 只要进入跟踪或短暂预测，就激活历史捕获标志

                # 读取当前最新的 IMU 绝对角度进行基准融合
                w_target_yaw, _ = imu.get_abs(vis_yaw, vis_pitch)
                w_target_pitch = vis_pitch
            else:
                # 纯视觉环的保命机制：彻底丢失目标时，立刻将 Pitch 误差清零刹车！
                w_target_pitch = 0.0
                pid_pitch.reset()  # 清空 PID 积分，防止残留的“力气”让电机乱窜

                if not has_tracked_ever:
                    # 【场景一】刚上电，从未抓到过靶子（解决评委任意放置导致背对靶子问题）
                    # 无条件以 3.0°/s 的速度单向匀速水平扫描
                    scan_dir = 1.0
                    w_target_yaw += scan_dir * yaw_scan_speed * loop_dt
                    w_target_yaw = (w_target_yaw + 180) % 360 - 180
                    print(f"--> [刚上电盲搜] 采用等效速度 3°/s 巡航扫描中... 目标世界角度: {w_target_yaw:.2f}°")
                
                else:
                    # 【场景二】行驶或拐弯中途跟跟丢了目标
                    # 提取卡尔曼滤波器最后记录的画面线速度方向趋势
                    if tracker.last_cy_vel != 0:
                        scan_dir = float(np.sign(tracker.last_cy_vel))
                    else:
                        scan_dir = 1.0 # 默认方向
                    
                    # 顺着最后丢失的方向，以 3.0°/s 的速度在世界坐标系下匀速扫掠
                    w_target_yaw += scan_dir * yaw_scan_speed * loop_dt
                    w_target_yaw = (w_target_yaw + 180) % 360 - 180
                    print(f"--> [动态盲搜] 顺着卡尔曼最后方向历史 {scan_dir} 以 3°/s 寻找... 目标世界角度: {w_target_yaw:.2f}°")

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