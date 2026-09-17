import time
import csv
import os
import math
import pandas as pd
import matplotlib.pyplot as plt
from robomaster import robot

# 1. กำหนด Root Directory สำหรับจัดเก็บไฟล์ให้เป็นระบบ
BASE_DIR = os.path.join("aj_thanate", "pid_lab_data")

chosen_scenario = 1
current_x = 0.0
current_y = 0.0
current_z = 0.0 

class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        self.prev_error = error
        output = p_term + i_term + d_term
        return max(self.min_output, min(output, self.max_output))

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

def get_timestamp():
    return time.time() * 1000

# ฟังก์ชันดึง Path ปลายทางแยกตามโฟลเดอร์สถานการณ์
def get_file_paths(scenario_num, is_pid):
    sit_dir = os.path.join(BASE_DIR, f"sit{scenario_num}")
    os.makedirs(sit_dir, exist_ok=True)
    filename = "position_with_pid.csv" if is_pid else "position_no_pid.csv"
    return os.path.join(sit_dir, filename)

# Callback สำหรับรับค่าพิกัดความถี่ 20 Hz และเขียนลงไฟล์ CSV
def position_handler(position_info):
    global current_x, current_y, current_z
    current_x, current_y, current_z = position_info  
    
    is_pid = True if chosen_scenario == 2 else False
    csv_path = get_file_paths(actual_sit_folder, is_pid)
    
    with open(csv_path, mode='a', newline='') as f:
        csv.writer(f).writerow([get_timestamp(), current_x, current_y])

def init_csv_headers(scenario_num, is_pid):
    csv_path = get_file_paths(scenario_num, is_pid)
    with open(csv_path, mode='w', newline='') as f:
        csv.writer(f).writerow(["Time", "X", "Y"])

# 📊 ฟังก์ชันพล็อตกราฟเดี่ยวแยกตามโหมดปฏิบัติการจริง (Single Mode Plotter)
def plot_results(sit_folder, is_pid):
    folder_path = os.path.join(BASE_DIR, f"sit{sit_folder}")
    
    # ดึงเฉพาะไฟล์ของโหมดที่เลือกกดรันในรอบนั้นมาพล็อต
    if is_pid:
        target_file = os.path.join(folder_path, "position_with_pid.csv")
        label_text = 'PID Mode (Closed-Loop)'
        line_color = 'blue'
        line_style = '-'
        img_name = "trajectory_plot_pid.png"
    else:
        target_file = os.path.join(folder_path, "position_no_pid.csv")
        label_text = 'Non-PID Mode (Open-Loop)'
        line_color = 'red'
        line_style = '--'
        img_name = "trajectory_plot_no_pid.png"

    print(f"\n📊 Generating Trajectory Plot for {label_text}...")
    
    if os.path.exists(target_file) and os.path.getsize(target_file) > 30:
        df = pd.read_csv(target_file)
        if not df.empty:
            plt.figure(figsize=(8, 8))
            
            # พล็อตเส้นทางเพียงโหมดเดียวเดี่ยว ๆ ตามต้องการ
            plt.plot(df['X'], df['Y'], label=label_text, color=line_color, alpha=0.8, linestyle=line_style)
            
            # ปักหมุดจุดเริ่มต้น (o) และจุดจบ (x)
            plt.scatter(df['X'].iloc[0], df['Y'].iloc[0], color=line_color, marker='o', s=100)
            plt.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color=line_color, marker='x', s=100)

            plt.title(f'RoboMaster 4 Laps Rectangle Trajectory ({label_text})')
            plt.xlabel('X Position (meters)')
            plt.ylabel('Y Position (meters)')
            plt.grid(True, linestyle=':', alpha=0.6)
            plt.axis('equal') 
            plt.legend()

            # บันทึกรูปภาพแยกชื่อโหมดและเปิดแสดงหน้าต่างกราฟ
            output_plot = os.path.join(folder_path, img_name)
            plt.savefig(output_plot, dpi=300)
            print(f"📈 Save plot image at: {output_plot}")
            plt.show()
    else:
        print("⚠ ไม่พบข้อมูลพิกัดในไฟล์ CSV หรือไฟล์ว่างเปล่า ไม่สามารถพล็อตกราฟได้")

if __name__ == '__main__':
    print("="*50)
    print(" RoboMaster Lab 04: One-Click Run & Plot System")
    print("="*50)
    print(" 1 : [🔴 Non-PID Mode] - วิ่งสี่เหลี่ยมผืนผ้า 4 รอบ (Open-Loop)")
    print(" 2 : [🔵 PID Mode]     - วิ่งสี่เหลี่ยมผืนผ้า 4 รอบด้วย drive_speed()")
    print("="*50)
    
    try:
        user_input = input("เลือกโหมดที่ต้องการรัน (1-2): ")
        chosen_scenario = int(user_input)
        if chosen_scenario not in [1, 2]: raise ValueError
    except ValueError:
        print("❌ กรุณากรอกเฉพาะตัวเลข 1 หรือ 2 เท่านั้น")
        exit()

    actual_sit_folder = 4  
    is_pid_mode = True if chosen_scenario == 2 else False
    init_csv_headers(actual_sit_folder, is_pid_mode)

    # 🛠️ [ตั้งค่าปรับจูนระยะทางและความเร็ว]
    DIST_X = 0.9   # ระยะทางเดินหน้า / ถอยหลัง (120 cm)
    DIST_Y = 0.9   # ระยะทางสไลด์ขวา / ซ้าย (60 cm)
    SPEED = 0.5    # ความเร็วขับเคลื่อนสูงสุด (m/s)
    
    ep_robot = robot.Robot()
    print(f"\nConnecting to RoboMaster...")
    ep_robot.initialize(conn_type="ap") 
    chassis = ep_robot.chassis

    chassis.sub_position(freq=20, callback=position_handler)
    time.sleep(1.5) 

    try:
        # ------------------------------------------------------------------
        # 🔴 โหมดที่ 1: วิ่งแบบไม่ใช้ PID (Open-Loop สี่เหลี่ยมผืนผ้า 4 รอบ)
        # ------------------------------------------------------------------
        if not is_pid_mode:
            print(f"\n>>> Running 4 Laps without PID...")
            for lap in range(1, 5):
                print(f"🔄 Lap {lap}/4")
                chassis.move(x=DIST_X, y=0, z=0, xy_speed=SPEED).wait_for_completed()
                time.sleep(0.4)
                chassis.move(x=0, y=DIST_Y, z=0, xy_speed=SPEED).wait_for_completed()
                time.sleep(0.4)
                chassis.move(x=-DIST_X, y=0, z=0, xy_speed=SPEED).wait_for_completed()
                time.sleep(0.4)
                chassis.move(x=0, y=-DIST_Y, z=0, xy_speed=SPEED).wait_for_completed()
                time.sleep(0.8) 

        # ------------------------------------------------------------------
        # 🔵 โหมดที่ 2: วิ่งแบบใช้ PID Control + drive_speed() (Closed-Loop 4 รอบ)
        # ------------------------------------------------------------------
        else:
            print(f"\n>>> Running 4 Laps with PID Control...")
            tolerance = 0.04  
            
            pid_x = PIDController(kp=1.8, ki=0.005, kd=0.15, min_output=-SPEED, max_output=SPEED)
            pid_y = PIDController(kp=1.8, ki=0.005, kd=0.15, min_output=-SPEED, max_output=SPEED)
            pid_z = PIDController(kp=0.7, ki=0.0, kd=0.05, min_output=-0.3, max_output=0.3)

            for lap in range(1, 5):
                print(f"🔄 Lap {lap}/4")
                
                start_lap_x = current_x
                start_lap_y = current_y
                
                waypoints = [
                    (start_lap_x + DIST_X, start_lap_y),            
                    (start_lap_x + DIST_X, start_lap_y + DIST_Y),   
                    (start_lap_x,          start_lap_y + DIST_Y),   
                    (start_lap_x,          start_lap_y)             
                ]

                for i, (target_x, target_y) in enumerate(waypoints):
                    pid_x.reset()
                    pid_y.reset()
                    pid_z.reset()
                    prev_time = time.time()

                    while True:
                        now = time.time()
                        dt = now - prev_time
                        prev_time = now
                        if dt <= 0: dt = 0.01

                        error_world_x = target_x - current_x
                        error_world_y = target_y - current_y
                        dist_error = (error_world_x**2 + error_world_y**2) ** 0.5
                        
                        if dist_error < tolerance:
                            chassis.drive_speed(x=0, y=0, z=0)
                            time.sleep(0.4) 
                            break

                        yaw_rad = math.radians(current_z)
                        error_robot_x = error_world_x * math.cos(yaw_rad) + error_world_y * math.sin(yaw_rad)
                        error_robot_y = -error_world_x * math.sin(yaw_rad) + error_world_y * math.cos(yaw_rad)

                        vx_cmd = pid_x.compute(setpoint=error_robot_x, measurement=0.0, dt=dt)
                        vy_cmd = pid_y.compute(setpoint=error_robot_y, measurement=0.0, dt=dt)
                        vz_cmd = pid_z.compute(setpoint=0.0, measurement=current_z, dt=dt)

                        chassis.drive_speed(x=vx_cmd, y=vy_cmd, z=vz_cmd)
                        
                        print(f"Lap:{lap} | Corner:{i+1} | Err:{dist_error:.3f}m | Yaw:{current_z:.1f}°", end='\r')
                        time.sleep(0.05)

        print(f"\n\nภารกิจสำเร็จ! ข้อมูลจัดเก็บลงโฟลเดอร์เรียบร้อยแล้ว")

    except KeyboardInterrupt:
        print("\n🛑 หยุดทำงานโดยผู้ใช้")
    finally:
        chassis.drive_speed(x=0, y=0, z=0)
        chassis.unsub_position()
        ep_robot.close()
        print("ปิดการทำงานระบบ RoboMaster เรียบร้อย!")
        
        # 🌟 ส่งตัวแปรควบคุมโหมดเข้าฟังก์ชันพล็อต เพื่อให้วาดกราฟเดี่ยวของโหมดนั้น ๆ
        plot_results(actual_sit_folder, is_pid_mode)