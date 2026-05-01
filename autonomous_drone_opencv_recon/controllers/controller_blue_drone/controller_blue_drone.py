import cv2
import numpy as np
import datetime
from controller import Robot

class Mavic2ProPrecisionRecon:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # 1. الحساسات
        self.imu = self.robot.getDevice("inertial unit"); self.imu.enable(self.time_step)
        self.gyro = self.robot.getDevice("gyro"); self.gyro.enable(self.time_step)
        self.gps = self.robot.getDevice("gps"); self.gps.enable(self.time_step)
        self.camera = self.robot.getDevice("camera"); self.camera.enable(self.time_step)
        self.camera_pitch_motor = self.robot.getDevice("camera pitch")
        
        # 2. المحركات
        self.motors = [self.robot.getDevice(n) for n in ["front left propeller", "front right propeller", "rear left propeller", "rear right propeller"]]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)

        # 3. الثوابت
        self.k_roll_p, self.k_roll_d = 30.0, 2.0
        self.k_pitch_p, self.k_pitch_d = 30.0, 2.0
        self.k_track_p = 0.01
        self.k_track_roll = 0.01
        self.k_track_yaw = 0.09
        
        self.target_altitude = 6.5  
        self.gimbal_pitch_value = 1.3 
        self.state = "TAKEOFF"
        self.target_captured = False # هذا المتغير يمنع التكرار، سأضيف إعادة ضبط له
        self.prev_altitude = 0.0

    def calculate_precise_coordinates(self, target_px_x, target_px_y, img_w, img_h):
        drone_coords = self.gps.getValues() 
        _, _, yaw = self.imu.getRollPitchYaw()
        drone_alt = drone_coords[2]
        
        fov_h = self.camera.getFov()
        fov_v = fov_h * (img_h / img_w)

        angle_off_x = ((target_px_x - (img_w / 2)) / (img_w / 2)) * (fov_h / 2)
        angle_off_y = ((target_px_y - (img_h / 2)) / (img_h / 2)) * (fov_v / 2)

        gimbal_tilt_error = 1.57 - self.gimbal_pitch_value
        corrected_angle_y = angle_off_y + gimbal_tilt_error

        dist_x = drone_alt * np.tan(angle_off_x)
        dist_y = drone_alt * np.tan(corrected_angle_y)

        # تحويل الإحداثيات العالمية
        world_x = drone_coords[0] + (dist_y * np.cos(yaw) - dist_x * np.sin(yaw))
        world_y = drone_coords[1] + (dist_y * np.sin(yaw) + dist_x * np.cos(yaw))

        return world_x, world_y

    def save_data(self, frame, target_pos, img_w, img_h):
        t_x, t_y = self.calculate_precise_coordinates(target_pos[0], target_pos[1], img_w, img_h)
        timestamp = datetime.datetime.now().strftime("%H%M%S")
        
        cv2.imwrite(f"capture_{timestamp}.png", frame)
        with open(f"report_{timestamp}.txt", "w", encoding="utf-8") as f:
            f.write(f"إحداثيات الهدف المحسوبة: X={t_x:.4f}, Y={t_y:.4f}\n")
            f.write(f"ارتفاع الطائرة: {self.gps.getValues()[2]:.2f}m\n")
        
        print(f"✅ تم الحفظ بنجاح في مجلد الكود! X:{t_x:.2f}, Y:{t_y:.2f}")

    def process_vision(self):
        raw_image = self.camera.getImage()
        if not raw_image: return None, 0, 0, None
        
        w, h = self.camera.getWidth(), self.camera.getHeight()
        frame = np.frombuffer(raw_image, np.uint8).reshape((h, w, 4))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 150, 50]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 150, 50]), np.array([180, 255, 255]))
        
        target_center = None
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 300:
                x, y, bw, bh = cv2.boundingRect(c)
                target_center = (x + bw//2, y + bh//2)
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)

        cv2.imshow("View", frame)
        cv2.waitKey(1)
        return target_center, w, h, frame

    def run(self):
        if self.camera_pitch_motor:
            self.camera_pitch_motor.setPosition(self.gimbal_pitch_value)

        while self.robot.step(self.time_step) != -1:
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            roll_rate, pitch_rate, yaw_rate = self.gyro.getValues()
            altitude = self.gps.getValues()[2]
            
            target_pos, img_w, img_h, frame = self.process_vision()
            
            # استقرار الارتفاع
            v_vel = (altitude - self.prev_altitude) / (self.time_step / 1000.0)
            self.prev_altitude = altitude
            vertical_input = 68.5 + (np.clip(self.target_altitude - altitude, -1.0, 1.0) * 10.0) - (v_vel * 5.0)

            desired_pitch, desired_roll, yaw_input = 0.0, 0.0, 0.0
            
            if self.state == "TAKEOFF":
                if altitude >= self.target_altitude - 0.2: self.state = "SCANNING"
            elif self.state == "SCANNING":
                desired_pitch = 0.06
                if target_pos: self.state = "TRACKING"
            elif self.state == "TRACKING":
                if not target_pos:
                    self.state = "SCANNING"
                    self.target_captured = False # تصفير الحالة للبحث عن هدف جديد
                else:
                    off_x = (target_pos[0] - (img_w / 2)) / (img_w / 2)
                    off_y = (target_pos[1] - (img_h / 2)) / (img_h / 2)
                    
                    yaw_input = -off_x * self.k_track_yaw
                    desired_roll = off_x * self.k_track_roll
                    desired_pitch = off_y * self.k_track_p

                    # شرط التقاط البيانات (جعلناه أسهل قليلاً بزيادة النطاق لـ 0.1)
                    if abs(off_x) < 0.1 and abs(off_y) < 0.1 and not self.target_captured:
                        self.save_data(frame, target_pos, img_w, img_h)
                        self.target_captured = True

            # مصفوفة المحركات الأصلية المضمونة لموديل Mavic
            r_i = (desired_roll - roll) * self.k_roll_p - (roll_rate * self.k_roll_d)
            p_i = (desired_pitch - pitch) * self.k_pitch_p - (pitch_rate * self.k_pitch_d)
            
            m1 = vertical_input + r_i - p_i - yaw_input
            m2 = vertical_input - r_i - p_i + yaw_input
            m3 = vertical_input + r_i + p_i + yaw_input
            m4 = vertical_input - r_i + p_i - yaw_input
            
            self.motors[0].setVelocity(m1)
            self.motors[1].setVelocity(-m2)
            self.motors[2].setVelocity(-m3)
            self.motors[3].setVelocity(m4)

controller = Mavic2ProPrecisionRecon()
controller.run()