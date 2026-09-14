import sys
import time
import heapq
import argparse
from collections import deque

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError:
    print("❌ ไม่พบไลบรารี robomaster กรุณาติดตั้ง หรือเช็ก venv")
    sys.exit(1)

# ==========================================
# 🗺️ 1. ข้อมูลแผนที่จากใบงาน Lab (4x4)
# ==========================================
grid = {
    (1, 4): 2, (2, 4): 3, (3, 4): 1, (4, 4): float('inf'),
    (1, 3): 2, (2, 3): float('inf'), (3, 3): 4, (4, 3): 2,
    (1, 2): 3, (2, 2): 1, (3, 2): float('inf'), (4, 2): 4,
    (1, 1): 2, (2, 1): 3, (3, 1): 2, (4, 1): 1
}

START = (1, 4)
GOAL = (4, 1)

# ทิศทาง: 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
DIR_MAPPING = {(0, 1): 0, (1, 0): 1, (0, -1): 2, (-1, 0): 3}
DIR_NAMES = ['NORTH (^)', 'EAST (>)', 'SOUTH (v)', 'WEST (<)']

# ==========================================
# 🧠 2. Pathfinding Algorithms
# ==========================================
def bfs(start, goal, map_grid):
    queue = deque([(start, [start], 0)])
    visited = set([start])
    while queue:
        current, path, total_cost = queue.popleft()
        if current == goal: return path, total_cost
        
        for dx, dy in DIRECTIONS:
            next_node = (current[0] + dx, current[1] + dy)
            if next_node in map_grid and map_grid[next_node] != float('inf') and next_node not in visited:
                visited.add(next_node)
                queue.append((next_node, path + [next_node], total_cost + map_grid[next_node]))
    return None, 0

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, map_grid):
    pq = [(0, 0, start, [start])]
    visited_g_cost = {start: 0}
    while pq:
        f, g, current, path = heapq.heappop(pq)
        if current == goal: return path, g

        for dx, dy in DIRECTIONS:
            next_node = (current[0] + dx, current[1] + dy)
            if next_node in map_grid and map_grid[next_node] != float('inf'):
                new_g = g + map_grid[next_node] 
                if next_node not in visited_g_cost or new_g < visited_g_cost[next_node]:
                    visited_g_cost[next_node] = new_g
                    f_score = new_g + heuristic(next_node, goal)
                    heapq.heappush(pq, (f_score, new_g, next_node, path + [next_node]))
    return None, 0

def draw_map(path, title):
    print(f"\n{'='*35}\n 🗺️ {title}\n{'='*35}")
    for y in range(4, 0, -1):
        row_str = f"y={y} | "
        for x in range(1, 5):
            pos = (x, y)
            if pos == START: row_str += "[ 🟢 ] "
            elif pos == GOAL: row_str += "[ 🚩 ] "
            elif grid[pos] == float('inf'): row_str += "[ ⬛️ ] "
            elif pos in path and pos != START and pos != GOAL: row_str += "[ ✨ ] "
            else: row_str += f"[ C{grid[pos]} ] "
        print(row_str)
    print("      ----------------------------\n       x=1    x=2    x=3    x=4\n")

# ==========================================
# 🤖 3. Robot Controller (Pure IMU PID)
# ==========================================
CELL_SIZE_MM = 600
BASE_SPEED = 0.25
TURN_SPEED = 50   
KP_YAW = 1.4      
KD_YAW = 0.08     
YAW_TARGETS = [0.0, 90.0, 180.0, -90.0]

class RobotController:
    def __init__(self, start_heading=0, sim=False):
        self.sim = sim
        self.current_yaw = 0.0
        self.tof_dist = 9999
        self.heading = start_heading 
        self.initial_yaw_offset = 0.0
        
        if not self.sim:
            self.ep = robot.Robot()
            print("\n[INFO] Connecting to RoboMaster via AP...")
            if not self.ep.initialize(conn_type="ap"): sys.exit(1)
            
            self.chassis = self.ep.chassis
            self.sensor = self.ep.sensor
            self.gimbal = self.ep.gimbal
            
            # 🌟 สั่งให้ Gimbal หันตรงไปข้างหน้า เพื่อใช้ ToF เป็นกันชน
            print("[INFO] ล็อกคอ Gimbal ให้อยู่ตรงกลาง...")
            self.gimbal.recenter().wait_for_completed(timeout=2)
            
            # Subscribe ค่าจาก ToF และ IMU
            self.sensor.sub_distance(freq=20, callback=self._tof_cb)
            self.chassis.sub_attitude(freq=20, callback=self._imu_cb)
            
            # รอรับข้อมูลชุดแรก
            time.sleep(1) 
            self.initial_yaw_offset = self.current_yaw - YAW_TARGETS[self.heading]
            print("\n✅ ระบบพร้อมลุย!")

    def _tof_cb(self, sub_info): self.tof_dist = sub_info[0]
    def _imu_cb(self, sub_info): self.current_yaw = sub_info[0]

    def normalize_angle(self, angle):
        while angle > 180.0: angle -= 360.0
        while angle <= -180.0: angle += 360.0
        return angle

    def turn_to(self, target_heading):
        if self.heading == target_heading: return
        turn_steps = (target_heading - self.heading) % 4
        
        if not self.sim:
            self.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.1)
            if turn_steps == 1: self.chassis.move(x=0, y=0, z=-90, z_speed=TURN_SPEED).wait_for_completed()
            elif turn_steps == 2: self.chassis.move(x=0, y=0, z=180, z_speed=TURN_SPEED).wait_for_completed()
            elif turn_steps == 3: self.chassis.move(x=0, y=0, z=90, z_speed=TURN_SPEED).wait_for_completed()
            time.sleep(0.3)
        self.heading = target_heading

    def move_forward_pid(self):
        if self.sim:
            print("   -> [SIM] เดินหน้า 1 บล็อก")
            time.sleep(0.5)
            return

        duration = (CELL_SIZE_MM / 1000) / BASE_SPEED
        t_start = time.time()
        last_yaw_error = 0.0
        target_yaw = self.initial_yaw_offset + YAW_TARGETS[self.heading]

        while time.time() - t_start < duration:
            # 🌟 ToF เป็นกันชน: ถ้าใกล้กว่า 150mm (15cm) ให้เบรกทันที
            if 0 < self.tof_dist <= 150:
                print(f"🚨 ระวัง! ToF ตรวจเจอสิ่งกีดขวางที่ระยะ {self.tof_dist}mm หยุดฉุกเฉิน!")
                break

            # รักษาเส้นทางให้ตรงเป๊ะด้วย IMU Yaw (ไม่สไลด์ออกข้างแล้ว)
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            d_yaw = yaw_error - last_yaw_error
            z_speed = (KP_YAW * yaw_error) + (KD_YAW * d_yaw)
            last_yaw_error = yaw_error

            # ขับเคลื่อนไปข้างหน้า (y_speed = 0)
            self.chassis.drive_speed(x=BASE_SPEED, y=0, z=z_speed)
            time.sleep(0.02)

        self.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.25)

    def close(self):
        if not self.sim:
            self.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            self.sensor.unsub_distance()
            self.chassis.unsub_attitude()
            self.ep.close()

# ==========================================
# 🚀 4. Main Execution
# ==========================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help="รันทดสอบบนคอม ไม่ต่อหุ่น")
    args = parser.parse_args()

    bfs_path, bfs_cost = bfs(START, GOAL, grid)
    draw_map(bfs_path, "BFS Algorithm (เน้นก้าวน้อย ไม่สน Cost)")
    print(f"🟩 BFS Path: {' -> '.join([str(p) for p in bfs_path])} (Cost: {bfs_cost})")

    astar_path, astar_cost = a_star(START, GOAL, grid)
    draw_map(astar_path, "A* Algorithm (เน้น Cost ต่ำที่สุด)")
    print(f"🟦 A* Path: {' -> '.join([str(p) for p in astar_path])} (Cost: {astar_cost})")

    print("\n" + "="*50)
    print(" 🎯 กรุณาเลือกเส้นทางที่ต้องการให้หุ่นยนต์เดินจริง")
    print(" [1] เดินตามเส้นทาง BFS (อาจจะฝ่าช่อง Cost แพง)")
    print(" [2] เดินตามเส้นทาง A* (อ้อมนิดหน่อยแต่ Cost ถูกกว่า)")
    print("="*50)
    
    choice = input("เลือก Algorithm (1 หรือ 2) [กด Enter เพื่อใช้ A*]: ").strip()
    
    if choice == '1':
        selected_path = bfs_path
        algo_name = "BFS Algorithm"
    else:
        selected_path = astar_path
        algo_name = "A* Algorithm"
        
    print(f"\n✅ ยืนยันใช้เส้นทาง: {algo_name}")
    print(f" ทิศทางเริ่มต้น: 0=NORTH(^), 1=EAST(>), 2=SOUTH(v), 3=WEST(<)")
    
    try:
        start_heading = int(input("ตอนนี้หุ่นยนต์หันหน้าไปทิศใด? (0-3) [ค่าเริ่มต้น 1(EAST)]: ") or 1) % 4
        input(f"\n🚀 กด [ENTER] เพื่อเริ่มลุยด้วย {algo_name} ...")
    except ValueError:
        start_heading = 1
        
    bot = RobotController(start_heading=start_heading, sim=args.sim)
    
    try:
        print(f"\n🚀 [START] กำลังเดินตามเส้นทาง {algo_name}...")
        for i in range(1, len(selected_path)):
            current_node = selected_path[i-1]
            next_node = selected_path[i]
            
            dx = next_node[0] - current_node[0]
            dy = next_node[1] - current_node[1]
            target_heading = DIR_MAPPING[(dx, dy)]
            
            print(f"📍 Step {i}: ไปที่ {next_node} | หันไปทิศ: {DIR_NAMES[target_heading]}")
            
            bot.turn_to(target_heading)
            bot.move_forward_pid()
            
        print("\n🏁 [MISSION ACCOMPLISHED] เดินถึงเป้าหมาย (Goal) เรียบร้อยแล้ว!")
        
    except KeyboardInterrupt:
        print("\n[INFO] ยกเลิกการทำงานโดยผู้ใช้")
    finally:
        bot.close()
        print("ปิดระบบปลอดภัย")

if __name__ == "__main__":
    main()