# utilities.py (修改后的版本)

from controller import Robot
import math

class MyCustomRobot(Robot):
    def __init__(self, verbose=True):
        """ Initializes the robot and the required devices. """
        super().__init__()
        self.MAX_WHEEL_SPEED = 3.0  # PR2 constant from pr2_demo.c
        self.WHEELS_DISTANCE = 0.4492 # PR2 constant from pr2_demo.c
        self.SUB_WHEELS_DISTANCE = 0.098 # PR2 constant from pr2_demo.c
        self.WHEEL_RADIUS = 0.08    # PR2 constant from pr2_demo.c
        self.ANGLE_THRESHOLD = 0.002 # 使用更小的角度阈值，提高旋转精度
        self.DISTANCE_THRESHOLD = 0.001 # 使用更小的距离阈值，提高移动精度
        self.DISTANCE_PER_GRID_CELL = 1.0 # This value needs to be adjusted based on your Webots world scaling
        self.timestep = 16 # Use PR2's timestep
        self.verbose = verbose
        self.current_angle_rad = 0.0 # Keep track of robot's orientation in radians
        self.speed_factor = 1.0  # 默认速度系数，用于障碍物避让
        
        # 新增：误差校验和纠偏参数
        self.position_error_threshold = 0.004  # 更小的位置误差阈值(米)
        self.angle_error_threshold = 0.0008   # 更小的角度误差阈值(弧度)
        self.correction_factor = 0.9          # 基础纠偏因子(0-1)
        self.position_correction_factor = 0.25 # 位置纠偏因子(小规模纠正)
        self.angle_correction_factor = 2.0    # 角度纠偏因子(大规模纠正) 
        self.max_correction_speed = 0.85      # 最大纠偏速度
        self.enable_error_correction = True   # 是否启用误差纠偏

        # PR2 specific wheel motors and sensors
        self.wheel_motors = {}
        self.wheel_sensors = {}
        self.rotation_motors = {}
        self.rotation_sensors = {}

        # 新增：PR2 机械臂、躯干和抓手电机字典
        self.arm_motors = {}
        self.arm_sensors = {}
        self.gripper_motors = {}
        self.gripper_sensors = {}
        self.torso_motor = None
        self.torso_sensor = None


    def initialize_devices(self):
        """ Initializes sensors and actuators for PR2. """
        self.iu = self.getDevice('imu_sensor') # Renamed based on pr2_demo.c
        if self.iu:
            self.iu.enable(self.timestep)
        self.gps = self.getDevice('gps') # Assuming a GPS is still available for position
        if self.gps:
            self.gps.enable(self.timestep)

        # Initialize PR2 wheel motors and sensors
        wheel_motor_names = [
            "fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
            "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
            "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
            "br_caster_l_wheel_joint", "br_caster_r_wheel_joint"
        ]
        for name in wheel_motor_names:
            motor = self.getDevice(name)
            if motor:
                self.wheel_motors[name] = motor
                self.wheel_sensors[name] = motor.getPositionSensor()
                if self.wheel_sensors[name]:
                    self.wheel_sensors[name].enable(self.timestep)
                self.wheel_motors[name].setPosition(float('inf'))
                self.wheel_motors[name].setVelocity(0.0)
            else:
                print(f"WARNING: Wheel motor '{name}' not found.")

        # Initialize PR2 rotation motors and sensors
        rotation_motor_names = [
            "fl_caster_rotation_joint", "fr_caster_rotation_joint",
            "bl_caster_rotation_joint", "br_caster_rotation_joint"
        ]
        for name in rotation_motor_names:
            motor = self.getDevice(name)
            if motor:
                self.rotation_motors[name] = motor
                self.rotation_sensors[name] = motor.getPositionSensor()
                if self.rotation_sensors[name]:
                    self.rotation_sensors[name].enable(self.timestep)
            else:
                print(f"WARNING: Rotation motor '{name}' not found.")

        # 新增：初始化机械臂和躯干电机
        # 左臂关节
        # left_arm_joint_names = [
        #     "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
        #     "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"
        # ]
        left_arm_joint_names = [
        ]
        # 右臂关节
        right_arm_joint_names = [
        ]
        # right_arm_joint_names = [
        #     "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
        #     "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"
        # ]
        # 抓手关节
        gripper_joint_names = []
        # gripper_joint_names = ["l_gripper_joint", "r_gripper_joint"]

        # 躯干关节
        torso_joint_name = "torso_lift_joint"

        all_arm_joint_names = left_arm_joint_names + right_arm_joint_names
        for name in all_arm_joint_names:
            motor = self.getDevice(name)
            if motor:
                self.arm_motors[name] = motor
                self.arm_sensors[name] = motor.getPositionSensor()
                if self.arm_sensors[name]:
                    self.arm_sensors[name].enable(self.timestep)
            else:
                print(f"WARNING: Arm motor '{name}' not found.")

        for name in gripper_joint_names:
            motor = self.getDevice(name)
            if motor:
                self.gripper_motors[name] = motor
                self.gripper_sensors[name] = motor.getPositionSensor()
                if self.gripper_sensors[name]:
                    self.gripper_sensors[name].enable(self.timestep)
            else:
                print(f"WARNING: Gripper motor '{name}' not found.")

        # self.torso_motor = self.getDevice(torso_joint_name)
        # if self.torso_motor:
        #     self.torso_sensor = self.torso_motor.getPositionSensor()
        #     if self.torso_sensor:
        #         self.torso_sensor.enable(self.timestep)
        # else:
        #     print(f"WARNING: Torso motor '{torso_joint_name}' not found.")

        # 确保没有机械臂自动伸展的代码在这里，如果有，请注释掉或删除。
        # 例如，之前你可能删除了 set_initial_position() 类似的调用。

        # Some devices, such as the InertialUnit, need some time to "warm up"
        self.wait(100) # Give some time for sensors to stabilize


    def wait(self, ms):
        """ Waits for a specified number of milliseconds. """
        self.step(ms)

    def set_wheels_speeds(self, fll, flr, frl, frr, bll, blr, brl, brr):
        """ Set the speeds of the robot wheels. """
        # ... (此部分保持不变) ...
        if self.wheel_motors.get("fl_caster_l_wheel_joint"):
            self.wheel_motors["fl_caster_l_wheel_joint"].setVelocity(fll)
        if self.wheel_motors.get("fl_caster_r_wheel_joint"):
            self.wheel_motors["fl_caster_r_wheel_joint"].setVelocity(flr)
        if self.wheel_motors.get("fr_caster_l_wheel_joint"):
            self.wheel_motors["fr_caster_l_wheel_joint"].setVelocity(frl)
        if self.wheel_motors.get("fr_caster_r_wheel_joint"):
            self.wheel_motors["fr_caster_r_wheel_joint"].setVelocity(frr)
        if self.wheel_motors.get("bl_caster_l_wheel_joint"):
            self.wheel_motors["bl_caster_l_wheel_joint"].setVelocity(bll)
        if self.wheel_motors.get("bl_caster_r_wheel_joint"):
            self.wheel_motors["bl_caster_r_wheel_joint"].setVelocity(blr)
        if self.wheel_motors.get("br_caster_l_wheel_joint"):
            self.wheel_motors["br_caster_l_wheel_joint"].setVelocity(brl)
        if self.wheel_motors.get("br_caster_r_wheel_joint"):
            self.wheel_motors["br_caster_r_wheel_joint"].setVelocity(brr)

    def set_wheels_speed(self, speed):
        """ Set all wheels to the same speed. """
        self.set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed)

    def stop_wheels(self):
        """ Stop all wheels. """
        self.set_wheels_speed(0.0)

    def set_rotation_wheels_angles(self, fl, fr, bl, br, wait_on_feedback):
        """ Set the rotation wheels angles. """
        # ... (此部分保持不变) ...
        if wait_on_feedback:
            self.stop_wheels()

        if self.rotation_motors.get("fl_caster_rotation_joint"):
            self.rotation_motors["fl_caster_rotation_joint"].setPosition(fl)
        if self.rotation_motors.get("fr_caster_rotation_joint"):
            self.rotation_motors["fr_caster_rotation_joint"].setPosition(fr)
        if self.rotation_motors.get("bl_caster_rotation_joint"):
            self.rotation_motors["bl_caster_rotation_joint"].setPosition(bl)
        if self.rotation_motors.get("br_caster_rotation_joint"):
            self.rotation_motors["br_caster_rotation_joint"].setPosition(br)

        if wait_on_feedback:
            target = [fl, fr, bl, br]
            rotation_sensor_keys = [
                "fl_caster_rotation_joint", "fr_caster_rotation_joint",
                "bl_caster_rotation_joint", "br_caster_rotation_joint"
            ]
            while self.step(self.timestep) != -1:
                all_reached = True
                for i, key in enumerate(rotation_sensor_keys):
                    if self.rotation_sensors.get(key) and \
                       not self.almost_equal(self.rotation_sensors[key].getValue(), target[i]):
                        all_reached = False
                        break
                if all_reached:
                    break

    def almost_equal(self, a, b, tolerance=0.05): # Based on pr2_demo.c TOLERANCE
        """ Check if two double values are almost equal. """
        return (a < b + tolerance) and (a > b - tolerance)

    def rotate_angle(self, angle_radians):
        """ Rotates the robot around itself of a given angle [rad] without real-time error correction. """
        if self.verbose:
            print(f"Rotating by {math.degrees(angle_radians):.2f} degrees without real-time error correction")

        # 记录起始角度和目标角度
        start_angle = self.get_current_angle_from_imu()
        target_angle = start_angle + angle_radians
        target_angle = self.normalize_angle(target_angle)
        
        if self.verbose:
            print(f"旋转起始角度: {math.degrees(start_angle):.2f}°, 目标角度: {math.degrees(target_angle):.2f}°")
        
        # 保存原始纠偏设置，以便完成后恢复
        original_error_correction = self.enable_error_correction
        
        # 关闭实时纠偏
        self.enable_error_correction = False
        
        self.stop_wheels()
        self.set_rotation_wheels_angles(3.0 * math.pi / 4.0, math.pi / 4.0, -3.0 * math.pi / 4.0, -math.pi / 4.0, True)

        max_wheel_speed = self.MAX_WHEEL_SPEED if angle_radians > 0 else -self.MAX_WHEEL_SPEED
        self.set_wheels_speed(max_wheel_speed)

        initial_wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue() if self.wheel_sensors.get("fl_caster_l_wheel_joint") else 0.0
        expected_travel_distance = abs(angle_radians * 0.5 * (self.WHEELS_DISTANCE + self.SUB_WHEELS_DISTANCE))
        correction_count = 0

        while self.step(self.timestep) != -1:
            if not self.wheel_sensors.get("fl_caster_l_wheel_joint"):
                break
                
            wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue()
            wheel0_travel_distance = abs(self.WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))

            if wheel0_travel_distance >= expected_travel_distance - self.DISTANCE_THRESHOLD:
                break
                
            # 不再进行实时角度误差检测和纠偏
            correction_count += 1
            
            # 接近目标时减速
            if expected_travel_distance - wheel0_travel_distance < 0.1 * expected_travel_distance and expected_travel_distance > 0.01:
                self.set_wheels_speed(0.2 * max_wheel_speed)

        self.set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
        self.stop_wheels()
        
        # 获取旋转后的实际角度（而不是计算的角度）
        final_angle = self.get_current_angle_from_imu()
        
        # 更新内部角度记录为实际测量角度（而不是计算得到的角度）
        # 这确保了内部记录的角度与实际角度一致
        self.current_angle_rad = final_angle
        
        # 计算与目标角度的误差
        final_angle_error = self.normalize_angle(final_angle - target_angle)
        
        # 检查最终角度误差是否大于阈值（2度），如果是则进行微调
        FINE_TUNING_THRESHOLD = math.radians(1.5)  # 2度的阈值
        if abs(final_angle_error) > FINE_TUNING_THRESHOLD:
            if self.verbose:
                print(f"最终角度误差较大: {math.degrees(final_angle_error):.2f}°，开始精细微调...")
            
            # 计算需要微调的角度
            fine_tuning_angle = final_angle_error
            
            # 设置更精细的旋转参数
            original_angle_correction_factor = self.angle_correction_factor
            original_max_correction_speed = self.max_correction_speed
            # original_error_correction已在函数开始处保存
            
            # 临时调整参数，使旋转更精细、速度更慢
            self.angle_correction_factor = self.angle_correction_factor * 0.5  # 减小修正因子
            self.max_correction_speed = self.max_correction_speed * 0.3        # 减小最大速度
            self.enable_error_correction = False  # 关闭实时误差纠偏，以便精细控制
            
            # 执行微调旋转
            self.stop_wheels()
            self.set_rotation_wheels_angles(3.0 * math.pi / 4.0, math.pi / 4.0, -3.0 * math.pi / 4.0, -math.pi / 4.0, True)
            
            # 计算微调旋转的轮子速度，使用较低的速度
            # 修正：当误差为正(fine_tuning_angle > 0)时，实际角度大于目标角度，需要逆时针旋转(负速度)
            # 当误差为负(fine_tuning_angle < 0)时，实际角度小于目标角度，需要顺时针旋转(正速度)
            fine_tuning_speed = (-self.MAX_WHEEL_SPEED * 0.2) if fine_tuning_angle > 0 else (self.MAX_WHEEL_SPEED * 0.2)
            
            if self.verbose:
                print(f"微调方向: {'逆时针(-)' if fine_tuning_speed < 0 else '顺时针(+)'}, " +
                      f"误差: {math.degrees(fine_tuning_angle):.2f}°, " +
                      f"目标角度: {math.degrees(target_angle):.2f}°, " +
                      f"当前角度: {math.degrees(final_angle):.2f}°")
                      
            self.set_wheels_speed(fine_tuning_speed)
            
            # 计算微调所需的轮子旋转距离
            fine_tuning_distance = abs(fine_tuning_angle * 0.5 * (self.WHEELS_DISTANCE + self.SUB_WHEELS_DISTANCE))
            initial_wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue() if self.wheel_sensors.get("fl_caster_l_wheel_joint") else 0.0
            
            # 执行微调旋转
            # 添加监控变量，用于检测微调效果
            start_tuning_time = self.getTime()
            max_tuning_time = 3.0  # 最大微调时间限制（秒）
            tuning_steps = 0
            initial_error = abs(final_angle_error)
            previous_error = initial_error
            
            while self.step(self.timestep) != -1:
                tuning_steps += 1
                if not self.wheel_sensors.get("fl_caster_l_wheel_joint"):
                    break
                
                # 检查是否超时
                current_time = self.getTime()
                if current_time - start_tuning_time > max_tuning_time:
                    if self.verbose:
                        print(f"微调超时终止: {max_tuning_time:.1f}秒")
                    break
                
                # 每隔几步检查微调效果
                if tuning_steps % 5 == 0:
                    current_angle = self.get_current_angle_from_imu()
                    current_error = abs(self.normalize_angle(current_angle - target_angle))
                    
                    if self.verbose and tuning_steps % 10 == 0:
                        print(f"微调进行中 - 当前误差: {math.degrees(current_error):.2f}°, 原始误差: {math.degrees(initial_error):.2f}°")
                    
                    # 如果误差已经很小，提前终止
                    if current_error < math.radians(1.0):
                        if self.verbose:
                            print(f"微调提前完成 - 误差已达标: {math.degrees(current_error):.2f}°")
                        break
                    
                    # 如果误差变大，提前终止
                    if current_error > previous_error * 1.1:  # 误差增大超过10%
                        if self.verbose:
                            print(f"微调终止 - 误差变大: {math.degrees(previous_error):.2f}° -> {math.degrees(current_error):.2f}°")
                        break
                    
                    previous_error = current_error
                
                wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue()
                wheel0_travel_distance = abs(self.WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))
                
                # 微调时使用更小的阈值，以提高精度
                if wheel0_travel_distance >= fine_tuning_distance - (self.DISTANCE_THRESHOLD * 0.5):
                    break
            
            # 完成微调后停止并重置轮子
            self.set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
            self.stop_wheels()
            
            # 再次验证角度
            final_angle_after_tuning = self.get_current_angle_from_imu()
            final_error_after_tuning = self.normalize_angle(final_angle_after_tuning - target_angle)
            
            if self.verbose:
                print(f"微调完成 - 调整前误差: {math.degrees(final_angle_error):.2f}°, 调整后误差: {math.degrees(final_error_after_tuning):.2f}°")
            
            # 检查微调是否有效果 - 如果误差变大，则恢复到微调前的状态
            if abs(final_error_after_tuning) > abs(final_angle_error) * 1.05:  # 误差增大超过5%
                if self.verbose:
                    print(f"微调无效: 误差变大，恢复到微调前状态")
                # 不更新内部角度记录，保留微调前的状态
                final_angle_error = final_angle_error  # 保持原误差
            else:
                # 微调有效，更新内部角度记录为实际测量角度
                self.current_angle_rad = final_angle_after_tuning
                final_angle_error = final_error_after_tuning
            
            # 恢复原始参数
            self.angle_correction_factor = original_angle_correction_factor
            self.max_correction_speed = original_max_correction_speed
            # self.enable_error_correction已由下面的代码恢复
            
            # 更新最终误差为微调后的误差
            final_angle_error = final_error_after_tuning
        
        # 恢复原始纠偏设置
        self.enable_error_correction = original_error_correction
        
        if self.verbose:
            print(f"旋转完成 - 目标角度: {math.degrees(target_angle):.2f}°, 实际角度: {math.degrees(final_angle):.2f}°")
            print(f"最终角度误差: {math.degrees(final_angle_error):.2f}°")
            if abs(final_angle_error) > self.angle_error_threshold:
                print(f"警告：最终角度误差较大！")


    def go_forward(self, distance):
        """ Moves the robot forward for a given distance [m] with real-time error correction. """
        if self.verbose:
            print(f"Moving forward by {distance:.2f} meters with error correction")

        # 记录起始位置和目标位置
        start_position = self.get_current_position()
        target_distance = abs(distance)
        direction = 1 if distance > 0 else -1
        
        # 计算目标位置
        target_position = [
            start_position[0] + distance * math.cos(self.current_angle_rad),
            start_position[1] + distance * math.sin(self.current_angle_rad)
        ]
        
        # 应用速度因子，在接近障碍物时减速
        max_wheel_speed = self.MAX_WHEEL_SPEED * direction * self.speed_factor
        self.set_wheels_speed(max_wheel_speed)
        
        initial_wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue() if self.wheel_sensors.get("fl_caster_l_wheel_joint") else 0.0
        correction_count = 0
        
        while self.step(self.timestep) != -1:
            if not self.wheel_sensors.get("fl_caster_l_wheel_joint"):
                break
                
            # 当前位置和行驶距离
            current_position = self.get_current_position()
            wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue()
            wheel0_travel_distance = abs(self.WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))
            
            # 检查是否到达目标距离
            if wheel0_travel_distance >= target_distance - self.DISTANCE_THRESHOLD:
                break
            
            # 实时角度误差检测和纠偏 - 只保留角度纠偏，移除距离实时纠偏
            if self.enable_error_correction and correction_count % 10 == 0:  # 每10个周期检查一次
                # 计算角度误差
                current_angle = self.get_current_angle_from_imu()
                angle_error = self.normalize_angle(current_angle - self.current_angle_rad)
                
                if self.verbose and abs(angle_error) > self.angle_error_threshold:
                    print(f"检测到角度误差: {math.degrees(angle_error):.2f}°")
                
                # 如果角度误差超过阈值，进行纠偏
                if abs(angle_error) > self.angle_error_threshold:
                    # 仅计算角度纠偏
                    _, angular_correction = self.calculate_correction_speeds(0, angle_error)
                    
                    # 应用角度纠偏 - 角度误差对左右轮差速的影响较大
                    base_speed = max_wheel_speed * 0.8  # 降低基础速度进行精确控制
                    
                    # 角度误差对左右轮差速的影响较大
                    left_speed = base_speed + angular_correction
                    right_speed = base_speed - angular_correction
                    
                    # 限制速度范围
                    left_speed = max(-self.MAX_WHEEL_SPEED, min(self.MAX_WHEEL_SPEED, left_speed))
                    right_speed = max(-self.MAX_WHEEL_SPEED, min(self.MAX_WHEEL_SPEED, right_speed))
                    
                    # 设置差速纠偏
                    self.set_wheels_speeds(left_speed, left_speed, right_speed, right_speed, 
                                         left_speed, left_speed, right_speed, right_speed)
                    
                    if self.verbose:
                        print(f"应用角度纠偏 - 左轮速度: {left_speed:.2f}, 右轮速度: {right_speed:.2f}")
                        print(f"  角度纠正(大): {angular_correction:.3f}")
                        
                    # 纠偏几个周期后恢复正常速度
                    for _ in range(5):
                        if self.step(self.timestep) == -1:
                            break
                    
                    # 恢复正常前进
                    self.set_wheels_speed(max_wheel_speed)
            
            correction_count += 1
            
            # 接近目标时减速 - 删除此功能

        self.stop_wheels()
        
        # 最终位置验证
        final_position = self.get_current_position()
        final_distance_error = self.calculate_distance(final_position, target_position)
        
        # 检查是否需要进行位置微调 - 如果误差大于0.1米
        POSITION_FINE_TUNING_THRESHOLD = 0.08  # 10厘米的阈值
        if final_distance_error > POSITION_FINE_TUNING_THRESHOLD:
            if self.verbose:
                print(f"移动完成后检测到位置误差较大: {final_distance_error:.3f}m > {POSITION_FINE_TUNING_THRESHOLD}m")
                print(f"目标位置: ({target_position[0]:.3f}, {target_position[1]:.3f}), 实际位置: ({final_position[0]:.3f}, {final_position[1]:.3f})")
                print("开始进行位置微调...")
            
            # 计算当前位置与目标位置在当前朝向上的投影距离
            current_heading = self.get_current_angle_from_imu()
            
            # 计算从当前位置到目标位置的向量
            dx = target_position[0] - final_position[0]
            dy = target_position[1] - final_position[1]
            
            # 计算在当前航向上的投影距离
            heading_vector_x = math.cos(current_heading)
            heading_vector_y = math.sin(current_heading)
            projected_distance = dx * heading_vector_x + dy * heading_vector_y
            
            if self.verbose:
                # 打印向量和投影距离以便调试
                vector_angle = math.degrees(math.atan2(dy, dx))
                heading_angle = math.degrees(current_heading)
                print(f"当前航向: {heading_angle:.2f}°, 目标方向: {vector_angle:.2f}°")
                print(f"投影距离: {projected_distance:.3f}m")
            
            # 如果投影距离不太小，进行微调
            if abs(projected_distance) > 0.03:  # 如果投影距离大于3厘米
                # 保存原始设置
                original_error_correction = self.enable_error_correction
                original_speed_factor = self.speed_factor
                
                # 关闭误差纠偏，降低速度
                self.enable_error_correction = False
                self.speed_factor = 0.3  # 30%的速度进行精细调整
                
                # 计算要移动的距离 - 使用投影距离的80%避免过调
                fine_tuning_distance = projected_distance * 0.8
                
                if self.verbose:
                    move_direction = "前进" if fine_tuning_distance > 0 else "后退"
                    print(f"位置微调: {move_direction} {abs(fine_tuning_distance):.3f}m")
                
                # 执行微调移动 - 沿当前朝向前进或后退
                if fine_tuning_distance > 0:
                    self.go_forward(fine_tuning_distance)  # 前进
                else:
                    self.go_backward(abs(fine_tuning_distance))  # 后退
                
                # 恢复原始设置
                self.enable_error_correction = original_error_correction
                self.speed_factor = original_speed_factor
                
                # 验证微调效果
                adjusted_position = self.get_current_position()
                adjusted_error = self.calculate_distance(adjusted_position, target_position)
                
                if self.verbose:
                    print(f"位置微调完成 - 调整前误差: {final_distance_error:.3f}m, 调整后误差: {adjusted_error:.3f}m")
                    
                    # 评估微调效果
                    if adjusted_error < final_distance_error:
                        improvement = ((final_distance_error - adjusted_error) / final_distance_error) * 100
                        print(f"微调有效，误差减少了 {improvement:.1f}%")
                    else:
                        print(f"微调效果不佳，误差反而增加了")
            else:
                if self.verbose:
                    print(f"投影距离较小 ({projected_distance:.3f}m)，不需要微调")
        elif self.verbose:
            print(f"运动完成 - 最终位置误差: {(final_distance_error * 100):.2f}cm (在可接受范围内)")
            
        # 即使不微调也记录最终状态，便于调试
        if self.verbose and final_distance_error > self.position_error_threshold:
            print(f"最终位置: 目标位置: ({target_position[0]:.3f}, {target_position[1]:.3f}), 实际位置: ({final_position[0]:.3f}, {final_position[1]:.3f})")


    def turn_east(self):
        """将机器人转向东方（绝对方向，0度）"""
        self._rotate_to_absolute_angle(0.0)

    def turn_north(self):
        """将机器人转向北方（绝对方向，90度）"""
        self._rotate_to_absolute_angle(math.pi / 2.0)

    def turn_west(self):
        """将机器人转向西方（绝对方向，180度）"""
        self._rotate_to_absolute_angle(math.pi)

    def turn_south(self):
        """将机器人转向南方（绝对方向，270度）"""
        self._rotate_to_absolute_angle(3.0 * math.pi / 2.0)

    def _calculate_and_rotate_to_target(self, target_angle):
        """基于当前角度计算相对旋转量
        已过时：请使用_rotate_to_absolute_angle代替
        """
        angle_diff = target_angle - self.current_angle_rad
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        self.rotate_angle(angle_diff)
        
    def _rotate_to_absolute_angle(self, absolute_angle):
        """将机器人旋转到指定的绝对角度
        
        Args:
            absolute_angle: 目标绝对角度（弧度）
        """
        # 获取当前的绝对角度（从IMU传感器）
        current_absolute_angle = self.get_current_angle_from_imu()
        
        # 计算需要旋转的角度差（考虑最短路径）
        angle_diff = absolute_angle - current_absolute_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        if self.verbose:
            print(f"旋转到绝对角度 - 当前: {math.degrees(current_absolute_angle):.2f}°, " +
                  f"目标: {math.degrees(absolute_angle):.2f}°, 需旋转: {math.degrees(angle_diff):.2f}°")
        
        # 执行旋转
        self.rotate_angle(angle_diff)
        
        # 更新内部角度记录为目标绝对角度
        self.current_angle_rad = absolute_angle

    # 新增：设置单个机械臂关节位置的方法 (保持不变)
    def set_arm_joint_position(self, joint_name, position, wait_on_feedback=True):
        motor = self.arm_motors.get(joint_name)
        sensor = self.arm_sensors.get(joint_name)
        if motor:
            # 确保设置的速度足够快，能够到达目标，但又不会过冲
            # PR2机械臂通常是位置控制模式，所以速度是最大速度
            # 如果不设置速度，它会使用默认的最大速度
            motor.setVelocity(motor.getMaxVelocity()) # 显式设置最大速度，以便快速到达目标
            motor.setPosition(position)
            if self.verbose:
                print(f"Setting {joint_name} to {position:.2f}")
            if wait_on_feedback and sensor:
                start_time = self.getTime()
                timeout = 2.0 # 给关节设置一个超时时间，防止无限等待
                while self.step(self.timestep) != -1:
                    # 检查是否超时
                    if self.getTime() - start_time > timeout:
                        print(f"WARNING: Joint '{joint_name}' timed out reaching position {position:.2f}. Current: {sensor.getValue():.2f}")
                        break
                    if self.almost_equal(sensor.getValue(), position, 0.01): # 更严格的关节位置阈值
                        break
        else:
            print(f"WARNING: Joint '{joint_name}' motor not found for setting position.")

    # 新增：设置躯干高度的方法 (保持不变，但请注意其对碰撞的影响)
    def set_torso_height(self, height, wait_on_feedback=True):
        if self.torso_motor:
            self.torso_motor.setVelocity(self.torso_motor.getMaxVelocity()) # 显式设置最大速度
            self.torso_motor.setPosition(height)
            if self.verbose:
                print(f"Setting torso height to {height:.2f}")
            if wait_on_feedback and self.torso_sensor:
                start_time = self.getTime()
                timeout = 2.0
                while self.step(self.timestep) != -1:
                    if self.getTime() - start_time > timeout:
                        print(f"WARNING: Torso timed out reaching height {height:.2f}. Current: {self.torso_sensor.getValue():.2f}")
                        break
                    if self.almost_equal(self.torso_sensor.getValue(), height, 0.01):
                        break
        else:
            print(f"WARNING: Torso motor not found for setting height.")

    # 新增：设置抓手的方法 (保持不变)
    def set_gripper_position(self, gripper_name, position, wait_on_feedback=True):
        motor = self.gripper_motors.get(gripper_name)
        sensor = self.gripper_sensors.get(gripper_name)
        if motor:
            motor.setVelocity(motor.getMaxVelocity()) # 显式设置最大速度
            motor.setPosition(position)
            if self.verbose:
                print(f"Setting {gripper_name} to {position:.2f}")
            if wait_on_feedback and sensor:
                start_time = self.getTime()
                timeout = 1.0 # 抓手可能更快
                while self.step(self.timestep) != -1:
                    if self.getTime() - start_time > timeout:
                        print(f"WARNING: Gripper '{gripper_name}' timed out reaching position {position:.2f}. Current: {sensor.getValue():.2f}")
                        break
                    if self.almost_equal(sensor.getValue(), position, 0.005):
                        break
        else:
            print(f"WARNING: Gripper motor '{gripper_name}' not found for setting position.")

    def retract_arms(self):
        """ 将PR2的两个机械臂收缩到一个安全的姿态。
            这些关节位置值需要你根据Webots中的PR2模型和环境手动调试。
            目标是找到一个既收缩又不会与任何障碍物（包括机器人自身）发生碰撞的姿态。
        """
        if self.verbose:
            print("Retracting PR2 arms to a safe posture...")

        # 1. 尝试将躯干降到最低，但如果这导致手臂碰撞，可能需要略微抬高
        #    PR2的躯干最低通常是 0.0，但为了避免某些组件碰撞地面，可以设为 0.01 或略高
        self.set_torso_height(0.05, True) # 尝试一个略微抬高但仍然很低的值，避免地面碰撞

        # 2. 设置左臂收缩姿态
        #    这些值需要你根据Webots调试得到
        #    一般策略：肩部向内收，抬高肘部，腕部弯曲，使手臂靠近身体
        #    以下是示例值，请根据你的仿真调整：
        self.set_arm_joint_position("l_shoulder_pan_joint", 0.0, False)    # 肩部平移，稍微向内收
        self.set_arm_joint_position("l_shoulder_lift_joint", 1.35, False)   # 肩部抬高，避免下垂碰撞
        self.set_arm_joint_position("l_upper_arm_roll_joint", 0.0, False) # 上臂滚动，保持中立或略微调整
        self.set_arm_joint_position("l_elbow_flex_joint", -2.2, False)    # 肘部弯曲，将前臂收回
        self.set_arm_joint_position("l_forearm_roll_joint", 0.0, False)   # 前臂滚动
        self.set_arm_joint_position("l_wrist_flex_joint", 0.0, False)     # 腕部弯曲
        self.set_arm_joint_position("l_wrist_roll_joint", 0.0, False)     # 腕部滚动

        # 3. 设置右臂收缩姿态 (镜像左臂，注意pan和roll关节的符号)
        #    通常，对于对称的机器人，右臂的pan和upper_arm_roll可能是左臂的负值
        self.set_arm_joint_position("r_shoulder_pan_joint", 0.0, False)   # 肩部平移，稍微向内收 (与左臂方向相反)
        self.set_arm_joint_position("r_shoulder_lift_joint", 1.35, False)   # 肩部抬高
        self.set_arm_joint_position("r_upper_arm_roll_joint", 0.0, False) # 上臂滚动
        self.set_arm_joint_position("r_elbow_flex_joint", -2.2, False)    # 肘部弯曲
        self.set_arm_joint_position("r_forearm_roll_joint", 0.0, False)
        self.set_arm_joint_position("r_wrist_flex_joint", 0.0, False)
        self.set_arm_joint_position("r_wrist_roll_joint", 0.0, False)

        # 4. 关闭抓手 (通常让它们完全闭合，避免手指伸出)
        self.set_gripper_position("l_gripper_joint", 0.0, False) # 0.0通常是闭合位置
        self.set_gripper_position("r_gripper_joint", 0.0, True)  # 最后一个等待反馈，确保所有机械臂都已到达目标位置

        if self.verbose:
            print("Arms retracted to safe posture.")

    def get_current_position(self):
        """获取当前GPS位置"""
        if self.gps:
            position = self.gps.getValues()
            return [position[0], position[1]]  # 返回[x, y]
        return [0.0, 0.0]
    
    def get_current_angle_from_imu(self):
        """从IMU获取当前角度(弧度)"""
        if self.iu:
            # IMU返回的是旋转矩阵，需要转换为角度
            rotation_matrix = self.iu.getRollPitchYaw()
            return rotation_matrix[2]  # Yaw角度
        return self.current_angle_rad
    
    def calculate_distance(self, pos1, pos2):
        """计算两点间距离
        pos2的坐标会先向最近的0.5的倍数四舍五入
        """
        # 将pos2的坐标向最近的0.5的倍数四舍五入
        rounded_pos2 = [
            round(pos2[0] * 2) / 2,  # 乘以2再四舍五入，再除以2，得到0.5的倍数
            round(pos2[1] * 2) / 2   # 同上
        ]
        # 使用四舍五入后的pos2计算距离
        return math.sqrt((pos1[0] - rounded_pos2[0])**2 + (pos1[1] - rounded_pos2[1])**2)
    
    def normalize_angle(self, angle):
        """将角度规范化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_correction_speeds(self, position_error, angle_error):
        """根据误差计算纠偏速度
        位置误差：小规模纠正 - 使用较小的修正因子
        偏航角误差：大规模纠正 - 使用较大的修正因子
        """
        # 位置纠偏：使用较小的修正因子，进行小规模纠正
        forward_correction = min(position_error * self.position_correction_factor, self.max_correction_speed * 0.5)
        
        # 角度纠偏：使用较大的修正因子，进行大规模纠正
        angular_correction = min(abs(angle_error) * self.angle_correction_factor, self.max_correction_speed)
        if angle_error < 0:
            angular_correction = -angular_correction
        
        if self.verbose and (position_error > self.position_error_threshold or abs(angle_error) > self.angle_error_threshold):
            print(f"纠偏计算 - 位置纠正: {forward_correction:.3f} (因子:{self.position_correction_factor:.2f}), " +
                 f"角度纠正: {angular_correction:.3f} (因子:{self.angle_correction_factor:.2f})")
            
        return forward_correction, angular_correction

    def set_error_correction_parameters(self, position_threshold=0.05, angle_threshold=0.1, 
                                       correction_factor=0.5, max_correction_speed=1.0, enable=True):
        """设置误差纠偏参数
        
        Args:
            position_threshold: 位置误差阈值(米)
            angle_threshold: 角度误差阈值(弧度)
            correction_factor: 纠偏因子(0-1)，越大纠偏越强
            max_correction_speed: 最大纠偏速度
            enable: 是否启用误差纠偏
        """
        self.position_error_threshold = position_threshold
        self.angle_error_threshold = angle_threshold
        self.correction_factor = correction_factor
        self.max_correction_speed = max_correction_speed
        self.enable_error_correction = enable
        
        # 计算派生的位置和角度纠偏因子 
        self.position_correction_factor = correction_factor * 0.25  # 位置小规模纠正
        self.angle_correction_factor = correction_factor * 2.0     # 角度大规模纠正
        
        if self.verbose:
            print(f"误差纠偏参数已更新:")
            print(f"  位置误差阈值: {position_threshold:.3f}m")
            print(f"  角度误差阈值: {math.degrees(angle_threshold):.2f}°")
            print(f"  基础纠偏因子: {correction_factor}")
            print(f"  位置纠偏因子: {self.position_correction_factor:.2f} (小规模纠正)")
            print(f"  角度纠偏因子: {self.angle_correction_factor:.2f} (大规模纠正)")
            print(f"  最大纠偏速度: {max_correction_speed}")
            print(f"  启用状态: {enable}")

    def go_to_position(self, target_x, target_y, tolerance=0.05):
        """精确移动到指定位置，带实时纠偏
        
        Args:
            target_x: 目标X坐标
            target_y: 目标Y坐标
            tolerance: 位置容差(米)
        """
        if self.verbose:
            print(f"精确移动到位置 ({target_x:.2f}, {target_y:.2f})")
        
        max_iterations = 50  # 最大迭代次数，避免无限循环
        iteration = 0
        
        while iteration < max_iterations:
            current_pos = self.get_current_position()
            current_angle = self.get_current_angle_from_imu()
            
            # 计算到目标的距离和角度
            dx = target_x - current_pos[0]
            dy = target_y - current_pos[1]
            distance_to_target = math.sqrt(dx*dx + dy*dy)
            
            # 检查是否到达目标
            if distance_to_target <= tolerance:
                if self.verbose:
                    print(f"已到达目标位置，最终误差: {distance_to_target:.3f}m")
                break
            
            # 计算需要转向的角度
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - current_angle)
            
            # 如果角度偏差较大，先转向
            if abs(angle_diff) > 0.1:  # 约5.7度
                if self.verbose:
                    print(f"调整方向，角度偏差: {math.degrees(angle_diff):.2f}°")
                self.rotate_angle(angle_diff)
                self.current_angle_rad = target_angle
            
            # 前进到目标位置
            move_distance = min(distance_to_target, 0.5)  # 每次最多移动0.5米
            if self.verbose:
                print(f"前进 {move_distance:.3f}m，剩余距离: {distance_to_target:.3f}m")
            self.go_forward(move_distance)
            
            iteration += 1
            
            # 短暂等待传感器稳定
            self.wait(50)
        
        if iteration >= max_iterations:
            current_pos = self.get_current_position()
            final_error = math.sqrt((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)
            print(f"警告: 达到最大迭代次数，最终位置误差: {final_error:.3f}m")

    def set_movement_speed_factor(self, factor):
        """
        设置移动速度系数，用于在接近障碍物时降低速度
        
        Args:
            factor: 速度系数 (0.0-1.0)，1.0 为全速
        """
        self.speed_factor = max(0.1, min(1.0, factor))  # 限制在 0.1-1.0 之间
        if self.verbose:
            print(f"速度系数已设置为: {self.speed_factor:.2f}")
            
    def get_movement_speed_factor(self):
        """
        获取当前速度系数
        
        Returns:
            当前速度系数
        """
        return self.speed_factor

    def go_backward(self, distance):
        """ Moves the robot backward for a given distance [m]. """
        if self.verbose:
            print(f"Moving backward by {distance:.2f} meters")
        
        # 向后移动就是向前移动负距离
        self.go_forward(-distance)

    def move_to_precise_position(self, target_pixel_position, scale_factor=20, max_attempts=3):
        """
        精确移动到指定的像素坐标位置
        
        Args:
            target_pixel_position: 目标像素坐标 (row, col)
            scale_factor: 缩放因子，用于像素到米的转换
            max_attempts: 最大尝试次数
            
        Returns:
            bool: 是否成功到达目标位置
        """
        if self.verbose:
            print(f"开始精确移动到像素坐标: {target_pixel_position}")
        
        # 获取当前GPS位置
        current_gps_position = self.get_current_position()
        
        # 将目标像素坐标转换为世界坐标(米)
        # 假设地图中心为原点，每个像素对应 DISTANCE_PER_GRID_CELL/scale_factor 米
        pixel_to_meter = self.DISTANCE_PER_GRID_CELL / scale_factor
        
        # 将像素坐标转换为相对于地图中心的米坐标
        # 注意：这里的转换可能需要根据你的具体地图配置进行调整
        target_world_x = (target_pixel_position[1] - 80) * pixel_to_meter  # 假设地图中心在(80, 80)像素
        target_world_y = -(target_pixel_position[0] - 80) * pixel_to_meter  # Y轴翻转
        target_world_position = (target_world_x, target_world_y)
        
        if self.verbose:
            print(f"目标世界坐标: ({target_world_x:.3f}, {target_world_y:.3f})米")
            print(f"当前世界坐标: ({current_gps_position[0]:.3f}, {current_gps_position[1]:.3f})米")
        
        success = False
        for attempt in range(max_attempts):
            # 计算距离和方向
            dx = target_world_position[0] - current_gps_position[0]
            dy = target_world_position[1] - current_gps_position[1]
            distance_to_target = math.sqrt(dx*dx + dy*dy)
            
            if self.verbose:
                print(f"尝试 {attempt + 1}/{max_attempts}: 距离目标 {distance_to_target:.3f}米")
            
            # 如果距离足够小，认为已到达
            if distance_to_target < 0.02:  # 2厘米精度
                if self.verbose:
                    print(f"精确定位成功！距离误差: {distance_to_target:.3f}米")
                success = True
                break
            
            # 计算目标方向角度
            target_angle = math.atan2(dy, dx)
            
            # 获取当前朝向
            current_angle = self.get_current_angle_from_imu()
            
            # 计算需要旋转的角度
            angle_diff = self.normalize_angle(target_angle - current_angle)
            
            # 保存原始设置
            original_error_correction = self.enable_error_correction
            original_speed_factor = self.speed_factor
            
            # 设置精确模式
            self.enable_error_correction = False
            self.speed_factor = 0.25  # 使用较低速度以提高精度
            
            # 如果角度差异较大，先转向
            if abs(angle_diff) > math.radians(5):  # 如果角度差大于5度
                if self.verbose:
                    print(f"调整朝向: 当前 {math.degrees(current_angle):.1f}°, 目标 {math.degrees(target_angle):.1f}°")
                self.rotate_angle(angle_diff)
                
                # 等待稳定
                for _ in range(10):
                    if self.step(self.timestep) == -1:
                        break
            
            # 直线移动到目标
            move_distance = min(distance_to_target, 0.2)  # 每次最多移动20厘米
            if self.verbose:
                print(f"向目标移动 {move_distance:.3f}米")
            
            self.go_forward(move_distance)
            
            # 恢复设置
            self.enable_error_correction = original_error_correction
            self.speed_factor = original_speed_factor
            
            # 更新当前位置
            current_gps_position = self.get_current_position()
            
            # 等待系统稳定
            for _ in range(20):
                if self.step(self.timestep) == -1:
                    break
        
        if not success:
            final_distance = self.calculate_distance(current_gps_position, target_world_position)
            if self.verbose:
                print(f"精确定位未完全成功，最终距离误差: {final_distance:.3f}米")
        
        return success

    def fine_tune_position_to_pixel(self, target_pixel_position, scale_factor=20, tolerance_pixels=2):
        """
        微调位置到指定像素坐标，专门用于最后的精确定位
        
        Args:
            target_pixel_position: 目标像素坐标 (row, col)
            scale_factor: 缩放因子
            tolerance_pixels: 容差像素数
            
        Returns:
            bool: 是否成功到达指定精度范围内
        """
        if self.verbose:
            print(f"开始像素级精确微调到: {target_pixel_position}, 容差: {tolerance_pixels}像素")
        
        # 获取当前位置
        current_position = self.get_current_position()
        
        # 将当前世界坐标转换回像素坐标以便比较
        pixel_to_meter = self.DISTANCE_PER_GRID_CELL / scale_factor
        
        # 转换当前位置为像素坐标（反向转换）
        current_pixel_col = int(current_position[0] / pixel_to_meter + 80)  # 加上地图中心偏移
        current_pixel_row = int(80 - current_position[1] / pixel_to_meter)  # Y轴翻转并加偏移
        current_pixel_position = (current_pixel_row, current_pixel_col)
        
        if self.verbose:
            print(f"当前像素坐标: {current_pixel_position}")
            print(f"目标像素坐标: {target_pixel_position}")
        
        # 计算像素级距离
        pixel_dx = target_pixel_position[1] - current_pixel_position[1]
        pixel_dy = target_pixel_position[0] - current_pixel_position[0]
        pixel_distance = math.sqrt(pixel_dx*pixel_dx + pixel_dy*pixel_dy)
        
        if self.verbose:
            print(f"像素距离: {pixel_distance:.1f}像素")
        
        # 如果距离在容差范围内，认为已到达
        if pixel_distance <= tolerance_pixels:
            if self.verbose:
                print(f"已在目标容差范围内! 像素距离: {pixel_distance:.1f} <= {tolerance_pixels}")
            return True
        
        # 计算需要移动的实际距离（米）
        actual_dx = pixel_dx * pixel_to_meter
        actual_dy = -pixel_dy * pixel_to_meter  # Y轴翻转
        actual_distance = math.sqrt(actual_dx*actual_dx + actual_dy*actual_dy)
        
        # 计算目标方向
        target_angle = math.atan2(actual_dy, actual_dx)
        current_angle = self.get_current_angle_from_imu()
        angle_diff = self.normalize_angle(target_angle - current_angle)
        
        if self.verbose:
            print(f"需要移动: ({actual_dx:.3f}, {actual_dy:.3f})米, 距离: {actual_distance:.3f}米")
            print(f"需要转向: {math.degrees(angle_diff):.1f}度")
        
        # 保存原始设置
        original_error_correction = self.enable_error_correction
        original_speed_factor = self.speed_factor
        
        # 设置超精确模式
        self.enable_error_correction = False
        self.speed_factor = 0.15  # 非常低的速度
        
        try:
            # 精确转向
            if abs(angle_diff) > math.radians(2):  # 如果角度差大于2度
                if self.verbose:
                    print("执行精确转向...")
                self.rotate_angle(angle_diff)
                
                # 等待稳定
                for _ in range(15):
                    if self.step(self.timestep) == -1:
                        break
            
            # 精确移动
            if actual_distance > 0.01:  # 如果距离大于1厘米
                if self.verbose:
                    print(f"执行精确移动: {actual_distance:.3f}米")
                self.go_forward(actual_distance)
                
                # 等待稳定
                for _ in range(15):
                    if self.step(self.timestep) == -1:
                        break
            
        finally:
            # 恢复设置
            self.enable_error_correction = original_error_correction
            self.speed_factor = original_speed_factor
        
        # 验证最终位置
        final_position = self.get_current_position()
        final_pixel_col = int(final_position[0] / pixel_to_meter + 80)
        final_pixel_row = int(80 - final_position[1] / pixel_to_meter)
        final_pixel_position = (final_pixel_row, final_pixel_col)
        
        final_pixel_dx = target_pixel_position[1] - final_pixel_position[1]
        final_pixel_dy = target_pixel_position[0] - final_pixel_position[0]
        final_pixel_distance = math.sqrt(final_pixel_dx*final_pixel_dx + final_pixel_dy*final_pixel_dy)
        
        success = final_pixel_distance <= tolerance_pixels
        
        if self.verbose:
            print(f"微调完成!")
            print(f"最终像素坐标: {final_pixel_position}")
            print(f"最终像素距离: {final_pixel_distance:.1f}像素")
            print(f"微调结果: {'成功' if success else '失败'}")
        
        return success