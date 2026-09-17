"""PID ตัวเดียวกับที่ใช้ในแล็บก่อนหน้า เก็บสำเนาไว้ให้ lab5 รันได้ด้วยตัวเอง"""


class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_output, self.max_output = min_output, max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.min_output, min(output, self.max_output))

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


class FilteredPID:
    """
    PID สำหรับสัญญาณที่มี noise มากอย่าง Sharp IR

    ต่างจาก PIDController ธรรมดาสี่อย่าง ซึ่งจำเป็นทั้งหมดเมื่อวัดด้วย IR
    ที่ 20 Hz

    1. กรองพจน์ D ด้วย low-pass
       อนุพันธ์ขยาย noise โดยตรง ค่า IR ที่กระเพื่อม 5 adc ระหว่างสองรอบ
       ที่ห่างกัน 0.05 วินาที คิดเป็นอนุพันธ์ 100 adc/s ถ้าไม่กรอง พจน์ D
       จะสั่งความเร็วกระตุกตามเสียงรบกวน ไม่ได้ตามการเคลื่อนที่จริง

    2. deadband
       ใกล้เป้าหมายแล้วปล่อยให้นิ่ง ไม่ต้องแก้ทุก adc ที่กระเพื่อม
       เป็นตัวหยุดอาการส่ายเล็ก ๆ รอบจุดสมดุลได้ตรงจุดที่สุด

    3. จำกัดอัตราการเปลี่ยนของ output (slew limit)
       กันไม่ให้คำสั่งความเร็วกระโดดจากสุดด้านหนึ่งไปสุดอีกด้านในรอบเดียว
       ซึ่งเป็นรูปแบบการแกว่งที่เห็นบ่อยที่สุดเวลาสัญญาณมีดีเลย์

    4. anti-windup
       หยุดสะสม I เมื่อ output ชนเพดานแล้ว ไม่งั้นค่าที่สะสมไว้จะดันหุ่น
       เลยเป้าไปอีกนานหลังกลับเข้าช่วงปกติ
    """

    def __init__(self, kp, ki, kd, out_limit,
                 deadband=0.0, d_alpha=0.3, slew_per_s=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.deadband = deadband
        self.d_alpha = d_alpha          # 0-1 ยิ่งน้อยยิ่งกรองหนัก
        self.slew_per_s = slew_per_s    # หน่วยของ output ต่อวินาที
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.d_filtered = 0.0
        self.prev_out = 0.0

    def compute(self, setpoint, measurement, dt):
        if dt <= 0:
            return self.prev_out

        error = setpoint - measurement
        if abs(error) <= self.deadband:
            error = 0.0
        else:
            error -= self.deadband if error > 0 else -self.deadband

        # D คิดจากผลต่างของ error แล้วกรองด้วย EMA
        if self.prev_error is None:
            raw_d = 0.0
        else:
            raw_d = (error - self.prev_error) / dt
        self.prev_error = error
        self.d_filtered += self.d_alpha * (raw_d - self.d_filtered)

        p_term = self.kp * error
        d_term = self.kd * self.d_filtered
        i_term = self.ki * self.integral

        out = p_term + i_term + d_term
        clamped = max(-self.out_limit, min(out, self.out_limit))

        # สะสม I เฉพาะตอนที่ยังไม่ชนเพดาน หรือตอนที่ error จะดึงกลับเข้าช่วง
        if self.ki != 0.0 and (out == clamped or (out - clamped) * error < 0):
            self.integral += error * dt

        if self.slew_per_s is not None:
            max_step = self.slew_per_s * dt
            delta = clamped - self.prev_out
            if delta > max_step:
                clamped = self.prev_out + max_step
            elif delta < -max_step:
                clamped = self.prev_out - max_step

        self.prev_out = clamped
        return clamped
