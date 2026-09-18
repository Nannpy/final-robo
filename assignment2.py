import os
import sys
import time
import json
import math
import csv
import argparse
import signal
import heapq
import tkinter as tk
from datetime import datetime

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError:
    print("❌ ไม่พบไลบรารี robomaster กรุณาติดตั้ง หรือเช็ก venv")
    sys.exit(1)

# ==========================================
# 🎯 1. Configuration & Constants
# ==========================================
GRID_W = 4 
GRID_H = 4 
CELL_SIZE_MM = 600

# พารามิเตอร์ระบบเดิน PID
BASE_SPEED = 0.25
TURN_SPEED = 50   
KP_YAW = 1.4      
KD_YAW = 0.08     
KP_WALL = 0.0025  
# 🌟 ปรับลดเพดานสไลด์ เพื่อให้หุ่นเดินหน้าเป็นหลัก ไม่แฉลบออกข้างมากเกินไป
MAX_CORRECTION_Y = 0.15 
YAW_TARGETS = [0.0, 90.0, 180.0, -90.0]

# พารามิเตอร์ Bayes Filter
L_MAX = 2.0
L_MIN = -2.0
P_OCC_GIVEN_DETECTION = 0.7
P_FREE_GIVEN_DETECTION = 0.3
L_OCC = math.log(P_OCC_GIVEN_DETECTION / (1 - P_OCC_GIVEN_DETECTION))
L_FREE = math.log(P_FREE_GIVEN_DETECTION / (1 - P_FREE_GIVEN_DETECTION))

DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
DIR_NAMES = ['NORTH (^)', 'EAST (>)', 'SOUTH (v)', 'WEST (<)']

# ==========================================
# 🗺️ 2. Map System 
# ==========================================
class MapSystem:
    def __init__(self):
        self.occ_grid = [[0.0 for _ in range(GRID_H)] for _ in range(GRID_W)]
        self.edges = {} 
        self.visited = set()
        self.history_log = []

    def log_odds_to_prob(self, l):
        return 1.0 - (1.0 / (1.0 + math.exp(l)))

    def update_cell(self, x, y, is_occ):
        if not (0 <= x < GRID_W and 0 <= y < GRID_H): return
        if (x, y) in self.visited: return 
        
        update_val = L_OCC if is_occ else L_FREE
        self.occ_grid[x][y] = max(L_MIN, min(L_MAX, self.occ_grid[x][y] + update_val))

    def mark_visited(self, x, y):
        self.visited.add((x, y))
        self.occ_grid[x][y] = L_MIN 

    def get_cell_status(self, x, y):
        if not (0 <= x < GRID_W and 0 <= y < GRID_H): return 'OUT'
        prob = self.log_odds_to_prob(self.occ_grid[x][y])
        if prob > 0.6: return 'OCC'
        if prob < 0.4: return 'FREE'
        return 'UNK'

    def set_edge(self, c1, c2, status):
        self.edges[tuple(sorted([c1, c2]))] = status

    def get_edge(self, c1, c2):
        return self.edges.get(tuple(sorted([c1, c2])), 0) 

# ==========================================
# 🖥️ 3. Advanced GUI Dashboard 
# ==========================================
class AdvancedGUI:
    def __init__(self, cell_px=110):
        self.root = tk.Tk()
        self.root.title("📡 RoboMaster God-Tier SLAM")
        self.root.configure(bg="#1E272E")
        
        self.cell_px = cell_px
        self.margin = 50
        
        self.map_w_px = GRID_W * cell_px + self.margin * 2
        self.map_h_px = GRID_H * cell_px + self.margin * 2
        self.panel_width = 320
        
        self.canvas = tk.Canvas(self.root, width=self.map_w_px + self.panel_width, height=self.map_h_px, bg="#2F3640", highlightthickness=0)
        self.canvas.pack()
        
        self.canvas.create_rectangle(self.map_w_px, 0, self.map_w_px + self.panel_width, max(self.map_h_px, 600), fill="#1E272E", outline="")
        self.canvas.create_text(self.map_w_px + 160, 30, text="🤖 TELEMETRY", fill="#00D2D3", font=("Arial", 16, "bold"))
        
        self.t_mode = self.canvas.create_text(self.map_w_px + 20, 80, text="MODE: INIT", fill="#FDCB6E", font=("Consolas", 12, "bold"), anchor="w")
        self.t_pos = self.canvas.create_text(self.map_w_px + 20, 110, text="POS: (0,0) | YAW: 0.0°", fill="#DFE6E9", font=("Consolas", 12), anchor="w")
        self.t_tof = self.canvas.create_text(self.map_w_px + 20, 150, text="ToF Front: --- mm", fill="#DFE6E9", font=("Consolas", 12), anchor="w")
        self.t_irl = self.canvas.create_text(self.map_w_px + 20, 180, text="IR Left  : ---", fill="#DFE6E9", font=("Consolas", 12), anchor="w")
        self.t_irr = self.canvas.create_text(self.map_w_px + 20, 210, text="IR Right : ---", fill="#DFE6E9", font=("Consolas", 12), anchor="w")
        self.t_step = self.canvas.create_text(self.map_w_px + 20, 250, text="Steps: 0", fill="#00B894", font=("Consolas", 14, "bold"), anchor="w")
        
        self.root.update()
        
    def update_map(self, robot, maps, mode_label="Exploring..."):
        try:
            self.canvas.delete("map_elements")
            
            for x in range(GRID_W):
                for y in range(GRID_H):
                    px1 = self.margin + x * self.cell_px
                    py1 = self.margin + (GRID_H - 1 - y) * self.cell_px
                    px2 = px1 + self.cell_px
                    py2 = py1 + self.cell_px
                    
                    status = maps.get_cell_status(x, y)
                    color = "#D63031" if status == 'OCC' else ("#00B894" if status == 'FREE' else "#636E72")
                    self.canvas.create_rectangle(px1, py1, px2, py2, fill=color, outline="#2D3436", tags="map_elements")
                    
                    prob = maps.log_odds_to_prob(maps.occ_grid[x][y])
                    text_color = "white" if status == 'OCC' else "#2D3436"
                    self.canvas.create_text(px1+self.cell_px/2, py1+self.cell_px/2, text=f"{prob:.2f}", fill=text_color, font=("Arial", 11, "bold"), tags="map_elements")

            for x in range(GRID_W):
                for y in range(GRID_H):
                    px1 = self.margin + x * self.cell_px
                    py1 = self.margin + (GRID_H - 1 - y) * self.cell_px
                    px2, py2 = px1 + self.cell_px, py1 + self.cell_px
                    
                    if maps.get_edge((x,y), (x,y+1)) == -1: self.canvas.create_line(px1, py1, px2, py1, fill="#FDCB6E", width=5, tags="map_elements")
                    if maps.get_edge((x,y), (x+1,y)) == -1: self.canvas.create_line(px2, py1, px2, py2, fill="#FDCB6E", width=5, tags="map_elements")
                    if x == 0 and maps.get_edge((x,y), (x-1,y)) == -1: self.canvas.create_line(px1, py1, px1, py2, fill="#FDCB6E", width=5, tags="map_elements")
                    if y == 0 and maps.get_edge((x,y), (x,y-1)) == -1: self.canvas.create_line(px1, py2, px2, py2, fill="#FDCB6E", width=5, tags="map_elements")

            rx = self.margin + robot.x * self.cell_px + self.cell_px/2
            ry = self.margin + (GRID_H - 1 - robot.y) * self.cell_px + self.cell_px/2
            rs = 20 
            if robot.heading == 0: pts = [rx, ry-rs, rx-rs, ry+rs, rx+rs, ry+rs] 
            elif robot.heading == 1: pts = [rx+rs, ry, rx-rs, ry-rs, rx-rs, ry+rs] 
            elif robot.heading == 2: pts = [rx, ry+rs, rx-rs, ry-rs, rx+rs, ry-rs] 
            elif robot.heading == 3: pts = [rx-rs, ry, rx+rs, ry-rs, rx+rs, ry+rs] 
            
            bot_color = "#0984E3" if "Navigating" in mode_label else "#E84393"
            self.canvas.create_polygon(pts, fill=bot_color, outline="white", width=2, tags="map_elements")
            
            self.canvas.itemconfigure(self.t_mode, text=f"MODE: {mode_label}", fill="#00D2D3" if "Explore" in mode_label else "#FDCB6E")
            self.canvas.itemconfigure(self.t_pos, text=f"POS: ({robot.x}, {robot.y}) | YAW: {DIR_NAMES[robot.heading][:5]}")
            self.canvas.itemconfigure(self.t_tof, text=f"ToF Front: {robot.tof_dist:4d} mm")
            
            ir_l, ir_r = robot.read_ir()
            self.canvas.itemconfigure(self.t_irl, text=f"IR Left  : {ir_l:4d}")
            self.canvas.itemconfigure(self.t_irr, text=f"IR Right : {ir_r:4d}")
            
            self.root.update()
        except Exception:
            pass

# ==========================================
# 🤖 4. Robot Controller (PID)
# ==========================================
class RobotController:
    def __init__(self, start_x=0, start_y=0, start_heading=0, sim=False):
        self.sim = sim
        self.current_yaw = 0.0
        self.tof_dist = 9999
        self.th_wall_l, self.th_free_l = 475, 450
        self.th_wall_r, self.th_free_r = 475, 450
        self.ref_sharp_l, self.ref_sharp_r = 530, 530
        self.initial_yaw_offset = 0.0
        
        self.x, self.y = start_x, start_y
        self.heading = start_heading 
        
        if not self.sim:
            self.ep = robot.Robot()
            print("[INFO] Connecting to RoboMaster via AP...")
            if not self.ep.initialize(conn_type="ap"): sys.exit(1)
            
            self.chassis = self.ep.chassis
            self.sensor = self.ep.sensor
            self.adaptor = self.ep.sensor_adaptor
            self.gimbal = self.ep.gimbal
            
            try: self.gimbal.recenter().wait_for_completed(timeout=2)
            except: pass

            self.adaptor.start()
            self.sensor.sub_distance(freq=20, callback=self._tof_cb)
            self.chassis.sub_attitude(freq=20, callback=self._imu_cb)
            time.sleep(1) 
        else:
            self.sim_walls = { 
                ((0,0), (1,0)), ((0,1), (1,1)), ((0,2), (1,2)),
                ((1,0), (2,0)), ((1,1), (2,1)), ((1,1), (1,2)),
                ((3,2), (3,3)), ((2,2), (3,2)), ((2,1), (3,1)), ((3,0), (3,1))
            }
            
    def _tof_cb(self, sub_info): self.tof_dist = sub_info[0]
    def _imu_cb(self, sub_info): self.current_yaw = sub_info[0]

    def read_ir(self):
        if self.sim:
            lx, ly = self.x + DIRS[(self.heading - 1) % 4][0], self.y + DIRS[(self.heading - 1) % 4][1]
            rx, ry = self.x + DIRS[(self.heading + 1) % 4][0], self.y + DIRS[(self.heading + 1) % 4][1]
            l_val = 550 if ((self.x, self.y), (lx, ly)) in self.sim_walls or ((lx, ly), (self.x, self.y)) in self.sim_walls or lx<0 or lx>=GRID_W else 350
            r_val = 550 if ((self.x, self.y), (rx, ry)) in self.sim_walls or ((rx, ry), (self.x, self.y)) in self.sim_walls or rx<0 or rx>=GRID_W else 350
            return l_val, r_val
        return self.adaptor.get_adc(id=1, port=1) or 0, self.adaptor.get_adc(id=2, port=1) or 0

    def read_tof(self):
        if self.sim:
            fx, fy = self.x + DIRS[self.heading][0], self.y + DIRS[self.heading][1]
            is_wall = ((self.x, self.y), (fx, fy)) in self.sim_walls or ((fx, fy), (self.x, self.y)) in self.sim_walls or fx<0 or fx>=GRID_W or fy<0 or fy>=GRID_H
            return 200 if is_wall else 800
        return self.tof_dist

    def normalize_angle(self, angle):
        while angle > 180.0: angle -= 360.0
        while angle <= -180.0: angle += 360.0
        return angle

    def calibrate(self):
        print("=" * 70)
        print(f" 3 วินาทีแรก: วางหุ่นตรงกึ่งกลางช่อง เพื่อจำ Reference ซ้าย-ขวา ")
        print("=" * 70)
        
        samples_r, samples_l = [], []
        t_start = time.time()
        
        while time.time() - t_start < 3.0:
            l, r = self.read_ir()
            if l > 0: samples_l.append(l)
            if r > 0: samples_r.append(r)
            rem_time = 3.0 - (time.time() - t_start)
            print(f"\r[CALIBRATING: {rem_time:.1f}s] Sharp-L: {l:4d} | Sharp-R: {r:4d} | ToF: {self.tof_dist:4d}mm", end="")
            time.sleep(0.05)
            
        self.ref_sharp_l = sum(samples_l)/len(samples_l) if samples_l else 530
        self.ref_sharp_r = sum(samples_r)/len(samples_r) if samples_r else 530
        self.initial_yaw_offset = self.current_yaw - YAW_TARGETS[self.heading]
        
        self.th_wall_l = self.ref_sharp_l - 15
        self.th_free_l = min(440, self.ref_sharp_l - 30)
        self.th_wall_r = self.ref_sharp_r - 15
        self.th_free_r = min(440, self.ref_sharp_r - 30)
        
        print(f"\n\n>>> Target R={self.ref_sharp_r:.0f} (Wall > {self.th_wall_r:.0f}) | Target L={self.ref_sharp_l:.0f} (Wall > {self.th_wall_l:.0f}) <<<")

    def reset_yaw(self):
        self.initial_yaw_offset = self.current_yaw - YAW_TARGETS[self.heading]

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
            self.x += DIRS[self.heading][0]; self.y += DIRS[self.heading][1]
            time.sleep(1); return

        duration = (CELL_SIZE_MM / 1000) / BASE_SPEED
        t_start = time.time()
        last_yaw_error = 0.0
        target_yaw = self.initial_yaw_offset + YAW_TARGETS[self.heading]

        while time.time() - t_start < duration:
            if 0 < self.tof_dist <= 150:
                break

            sharp_l, sharp_r = self.read_ir()
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            d_yaw = yaw_error - last_yaw_error
            z_speed = (KP_YAW * yaw_error) + (KD_YAW * d_yaw)
            last_yaw_error = yaw_error

            has_wall_r = (sharp_r >= self.th_wall_r)
            has_wall_l = (sharp_l >= self.th_wall_l)

            # 🌟 [ดึงกลับมาใช้ลอจิกเดิมของคุณเป๊ะๆ เพื่อให้เดินหน้าเป็นหลัก ไม่สไลด์แฉลบ!]
            if has_wall_r and has_wall_l:
                diff = (sharp_l - self.ref_sharp_l) - (sharp_r - self.ref_sharp_r)
                y_speed = KP_WALL * diff
            elif has_wall_r:
                err_r = self.ref_sharp_r - sharp_r
                y_speed = KP_WALL * err_r
            elif has_wall_l:
                err_l = self.ref_sharp_l - sharp_l
                y_speed = -KP_WALL * err_l
            else:
                y_speed = 0.0

            # 🌟 ตัวล็อกความเร็วสไลด์ ไม่ให้ขโมยซีนความเร็วเดินหน้า
            y_speed = max(min(y_speed, MAX_CORRECTION_Y), -MAX_CORRECTION_Y)
            
            self.chassis.drive_speed(x=BASE_SPEED, y=y_speed, z=z_speed)
            time.sleep(0.02)

        self.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.25)
        
        self.x += DIRS[self.heading][0]
        self.y += DIRS[self.heading][1]

    def close(self):
        if not self.sim:
            self.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            self.sensor.unsub_distance()
            self.chassis.unsub_attitude()
            self.adaptor.stop()
            self.ep.close()

# ==========================================
# 🧠 5. A-Star Algorithm 
# ==========================================
def a_star_planner(maps, start_x, start_y, start_heading, target_mode="explore", target_pos=None):
    pq = [(0, start_x, start_y, start_heading, [(start_x, start_y)])]
    visited_states = set() 
    
    while pq:
        cost, cx, cy, c_head, path = heapq.heappop(pq)
        
        if target_mode == "explore":
            if (cx, cy) not in maps.visited and (cx, cy) != (start_x, start_y): return path
        else:
            if (cx, cy) == target_pos: return path
                
        state = (cx, cy, c_head)
        if state in visited_states: continue
        visited_states.add(state)
        
        for d_idx in range(4):
            nx, ny = cx + DIRS[d_idx][0], cy + DIRS[d_idx][1]
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                e_stat = maps.get_edge((cx, cy), (nx, ny))
                if e_stat != -1: 
                    move_cost = 1.0 if e_stat == 1 else 5.0 
                    turn_diff = (d_idx - c_head) % 4
                    turn_penalty = 0.0 if turn_diff == 0 else (1.5 if turn_diff == 2 else 0.8) 
                    
                    new_cost = cost + move_cost + turn_penalty
                    h = abs(target_pos[0] - nx) + abs(target_pos[1] - ny) if target_pos else 0
                        
                    heapq.heappush(pq, (new_cost + h, nx, ny, d_idx, path + [(nx, ny)]))
    return None

# ==========================================
# 💾 6. Output Handling
# ==========================================
def save_results(maps, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    with open(f"{out_dir}/lab_results.csv", 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Step", "Robot Pos(x,y)", "IR Left", "Front(ToF)", "IR Right", "Map Status (L,F,R)"])
        writer.writerows(maps.history_log)
    with open(f"{out_dir}/state.json", 'w') as f:
        json.dump({"occ_grid": maps.occ_grid, "edges": [{"c1": e[0], "c2": e[1], "status": s} for e, s in maps.edges.items()]}, f, indent=2)
    print(f"\n[INFO] Data saved to {out_dir}")

# ==========================================
# 🚀 7. Main Execution Loop
# ==========================================
def main():
    global GRID_W, GRID_H
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help="Run in simulation mode")
    args = parser.parse_args()

    print("\n" + "="*54)
    print("      🌐 MAP & START POSITION SETUP")
    print("="*54)
    print("💡 เคล็ดลับ: ถ้าไม่รู้ขนาดแผนที่ ให้ใส่เลขเผื่อไว้ (เช่น 6x6) หุ่นจะหารั้วกำแพงเอง!")
    try:
        w_in = input("ใส่ความกว้างของแผนที่ (แกน X) [กด Enter เพื่อใช้ 4]: ")
        GRID_W = int(w_in) if w_in else 4
        h_in = input("ใส่ความยาวของแผนที่ (แกน Y) [กด Enter เพื่อใช้ 4]: ")
        GRID_H = int(h_in) if h_in else 4
        
        start_x = int(input(f"\nใส่พิกัดเริ่มต้น แกน X (0-{GRID_W-1}) [กด Enter เพื่อใช้ 0]: ") or 0)
        start_y = int(input(f"ใส่พิกัดเริ่มต้น แกน Y (0-{GRID_H-1}) [กด Enter เพื่อใช้ 0]: ") or 0)
        print("\nทิศทาง: 0=NORTH(^), 1=EAST(>), 2=SOUTH(v), 3=WEST(<)")
        start_heading = int(input("หุ่นยนต์หันหน้าไปทิศใด? (0-3) [กด Enter เพื่อใช้ 0]: ") or 0) % 4
    except ValueError:
        print("❌ ข้อมูลไม่ถูกต้อง ใช้ค่าเริ่มต้น (4x4, เริ่มที่ 0,0 ทิศ 0)")
        GRID_W, GRID_H, start_x, start_y, start_heading = 4, 4, 0, 0, 0

    out_dir = f"results_{'SIM' if args.sim else 'REAL'}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    robot = RobotController(start_x=start_x, start_y=start_y, start_heading=start_heading, sim=args.sim)
    maps = MapSystem()
    gui = AdvancedGUI()
    
    def safe_exit(sig, frame):
        print("\n[WARN] Ctrl+C detected. Saving...")
        save_results(maps, out_dir); robot.close(); sys.exit(0)
    signal.signal(signal.SIGINT, safe_exit)

    for i in range(GRID_W):
        maps.set_edge((i, 0), (i, -1), -1); maps.set_edge((i, GRID_H-1), (i, GRID_H), -1)
    for i in range(GRID_H):
        maps.set_edge((0, i), (-1, i), -1); maps.set_edge((GRID_W-1, i), (GRID_W, i), -1)

    robot.calibrate()
    step = 0
    
    # ---------------------------------------------------------
    # PHASE 1: EXPLORATION 
    # ---------------------------------------------------------
    print("\n[INFO] 🗺️ PHASE 1: Autonomous Exploration Started...\n")
    
    while True:
        step += 1
        x, y = robot.x, robot.y
        maps.mark_visited(x, y)
        gui.canvas.itemconfigure(gui.t_step, text=f"Steps: {step}")
        
        ir_l, ir_r = robot.read_ir()
        tof_f = robot.read_tof()
        
        l_dir, r_dir = (robot.heading - 1) % 4, (robot.heading + 1) % 4
        lx, ly = x + DIRS[l_dir][0], y + DIRS[l_dir][1]
        rx, ry = x + DIRS[r_dir][0], y + DIRS[r_dir][1]
        
        stat_l = 'OCC' if ir_l > robot.th_wall_l else ('FREE' if ir_l < robot.th_free_l else 'UNK')
        stat_r = 'OCC' if ir_r > robot.th_wall_r else ('FREE' if ir_r < robot.th_free_r else 'UNK')
        
        if stat_l != 'UNK': 
            maps.update_cell(lx, ly, stat_l == 'OCC')
            if stat_l == 'FREE' and (lx, ly) not in maps.visited: maps.set_edge((x,y), (lx,ly), 1)
        if stat_r != 'UNK': 
            maps.update_cell(rx, ry, stat_r == 'OCC')
            if stat_r == 'FREE' and (rx, ry) not in maps.visited: maps.set_edge((x,y), (rx,ry), 1)

        maps.history_log.append([step, f"({x},{y})", ir_l, tof_f, ir_r, f"{stat_l}, N/A, {stat_r}"])
        
        mode_lbl = "Exploration (A*)"
        gui.update_map(robot, maps, mode_lbl)
        
        path = a_star_planner(maps, x, y, robot.heading, target_mode="explore")
        
        if not path:
            print("\n🎉 [SUCCESS] แผนที่ถูกสำรวจครบหมดแล้ว! (Map Fully Explored)")
            break 
            
        next_node = path[1]
        target_dir = DIRS.index((next_node[0] - x, next_node[1] - y))
        robot.turn_to(target_dir)
        gui.update_map(robot, maps, mode_lbl)
        
        tof_verify = robot.read_tof()
        if tof_verify < 400:
            print(f"🚨 [WARN] ToF เจอกำแพงที่ {next_node}. อัปเดตแผนที่และหาทางใหม่...")
            maps.set_edge((x,y), next_node, -1)  
            maps.update_cell(next_node[0], next_node[1], True)
            continue
        else:
            maps.set_edge((x,y), next_node, 1) 
            robot.move_forward_pid()

    # ---------------------------------------------------------
    # PHASE 2: CUSTOM NAVIGATION 
    # ---------------------------------------------------------
    robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    print("\n" + "="*54)
    print(" 🚀 PHASE 2: CUSTOM NAVIGATION MODE")
    print("="*54)
    print("คุณสามารถยกหุ่นยนต์ไปวางที่ 'จุดเริ่มต้นใหม่' ได้เลย")
    
    try:
        new_start_x = int(input(f"ใส่พิกัดเริ่มต้นใหม่ แกน X (0-{GRID_W-1}): "))
        new_start_y = int(input(f"ใส่พิกัดเริ่มต้นใหม่ แกน Y (0-{GRID_H-1}): "))
        new_heading = int(input("หุ่นยนต์หันหน้าไปทิศใด? (0-3): ")) % 4
        
        goal_x = int(input(f"ใส่เป้าหมายที่ต้องการไป แกน X (0-{GRID_W-1}): "))
        goal_y = int(input(f"ใส่เป้าหมายที่ต้องการไป แกน Y (0-{GRID_H-1}): "))
    except ValueError:
        print("❌ ข้อมูลไม่ถูกต้อง โปรแกรมจะปิดและบันทึกผลการสำรวจ")
        save_results(maps, out_dir); robot.close(); sys.exit(0)

    robot.x, robot.y, robot.heading = new_start_x, new_start_y, new_heading
    robot.reset_yaw() 
    GOAL_POS = (goal_x, goal_y)
    
    print(f"\n[INFO] 🚦 นำทางจาก ({new_start_x},{new_start_y}) มุ่งหน้าสู่เป้าหมาย {GOAL_POS}...")
    
    while True:
        gui.canvas.itemconfigure(gui.t_step, text=f"Navigating")
        mode_lbl = f"Navigating to {GOAL_POS} (A*)"
        gui.update_map(robot, maps, mode_lbl)
        
        if (robot.x, robot.y) == GOAL_POS:
            print(f"\n🏁 [MISSION ACCOMPLISHED] ถึงจุดหมายปลายทาง {GOAL_POS} แล้ว!")
            break
            
        path = a_star_planner(maps, robot.x, robot.y, robot.heading, target_mode="return", target_pos=GOAL_POS)
        if not path:
            print(f"\n❌ [ERROR] ไม่มีเส้นทางที่เป็นไปได้จากแผนที่ที่สำรวจมา!")
            break

        next_node = path[1]
        target_dir = DIRS.index((next_node[0] - robot.x, next_node[1] - robot.y))
        
        robot.turn_to(target_dir)
        gui.update_map(robot, maps, mode_lbl)
        
        tof_verify = robot.read_tof()
        if tof_verify < 400:
            print(f"🚨 [WARN] เห้ย! มีสิ่งกีดขวางโผล่มาใหม่ที่ {next_node}. อัปเดตแผนที่และหาทางอ้อม...")
            maps.set_edge((robot.x, robot.y), next_node, -1)  
            maps.update_cell(next_node[0], next_node[1], True)
            continue
        else:
            robot.move_forward_pid()

    save_results(maps, out_dir)
    robot.close()
    print("\n[INFO] Shutting down systems. You may close the GUI.")
    gui.root.mainloop()

if __name__ == "__main__":
    main()