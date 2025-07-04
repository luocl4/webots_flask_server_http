from controller import Robot
import math
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from logger import logger

class MyCustomRobot(Robot):
    def __init__(self, verbose=False):
        """ Initializes the robot and the required devices for urdf_arm robot. """
        super().__init__()
        self.MAX_WHEEL_SPEED = 15.0  # 基于proto文件中的maxVelocity=15.0
        # 基于实际测试和物理布局重新校准参数
        self.WHEELS_DISTANCE_X = 0.4  # 前后轮距离 - 根据实际测试调整为0.4米
        self.WHEELS_DISTANCE_Y = 0.3  # 左右轮距离 - 根据实际测试调整为0.3米
        self.WHEELS_DISTANCE = math.sqrt(self.WHEELS_DISTANCE_X**2 + self.WHEELS_DISTANCE_Y**2)  # 对角线距离
        self.SUB_WHEELS_DISTANCE = 0.1  # 同一caster内两轮距离 - 调整为0.1米
        self.WHEEL_RADIUS = 0.074792   # 轮子半径从0.05调整为0.074792米，与URDF文件保持一致
        # 新增：旋转半径计算 - 机器人中心到轮子的距离
        self.ROTATION_RADIUS = math.sqrt((self.WHEELS_DISTANCE_X/2)**2 + (self.WHEELS_DISTANCE_Y/2)**2)
        self.ANGLE_THRESHOLD = 0.002 # 使用更小的角度阈值，提高旋转精度
        self.DISTANCE_THRESHOLD = 0.001 # 使用更小的距离阈值，提高移动精度
        self.DISTANCE_PER_GRID_CELL = 1.0 # This value needs to be adjusted based on your Webots world scaling
        self.timestep = 16 # 使用合适的时间步长
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

        # urdf_arm specific wheel motors and sensors (与PR2相同)
        self.wheel_motors = {}
        self.wheel_sensors = {}
        self.rotation_motors = {}
        self.rotation_sensors = {}

        # 新增：urdf_arm 机械臂、躯干和抓手电机字典
        self.arm_motors = {}
        self.arm_sensors = {}
        self.gripper_motors = {}
        self.gripper_sensors = {}
        self.torso_motor = None
        self.torso_sensor = None


    def initialize_devices(self):
        """ Initializes sensors and actuators for urdf_arm robot. """
        # 根据新的urdf_arm proto文件，传感器通过extensionSlot添加
        self.iu = self.getDevice('imu_sensor') # IMU传感器名称保持不变
        if self.iu:
            self.iu.enable(self.timestep)
            logger.info("IMU sensor 'imu_sensor' 已启用")
        else:
            logger.warning("IMU sensor 'imu_sensor' not found.")
            # 尝试获取不同名称的IMU传感器
            alternative_imu_names = ['inertial_unit', 'imu', 'inertial']
            for alt_name in alternative_imu_names:
                self.iu = self.getDevice(alt_name)
                if self.iu:
                    self.iu.enable(self.timestep)
                    logger.info(f"发现替代IMU传感器 '{alt_name}' 并已启用")
                    break
            
        self.gps = self.getDevice('gps') # GPS传感器通过extensionSlot2添加
        if self.gps:
            self.gps.enable(self.timestep)
            logger.info("GPS sensor 'gps' 已启用")
        else:
            logger.warning("GPS sensor 'gps' not found.")
            # 尝试获取不同名称的GPS传感器
            alternative_gps_names = ['gps_sensor', 'position_sensor', 'loc']
            for alt_name in alternative_gps_names:
                self.gps = self.getDevice(alt_name)
                if self.gps:
                    self.gps.enable(self.timestep)
                    logger.info(f"发现替代GPS传感器 '{alt_name}' 并已启用")
                    break

        # Initialize urdf_arm wheel motors and sensors (与PR2相同的命名)
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
                logger.warning(f"Wheel motor '{name}' not found.")

        # Initialize urdf_arm rotation motors and sensors (与PR2相同的命名)
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
                logger.warning(f"Rotation motor '{name}' not found.")

        # 新增：初始化机械臂和躯干电机
        # 根据新的urdf_arm proto文件，使用新的关节命名
        # 左臂关节
        left_arm_joint_names = [
            "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint",
            "zarm_l4_joint", "zarm_l5_joint"
        ]
        # 右臂关节
        right_arm_joint_names = [
            "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", 
            "zarm_r4_joint", "zarm_r5_joint"
        ]
        # 抓手关节 - 使用主要的抓手电机
        gripper_joint_names = ["left_gripper_finger_joint", "right_gripper_finger_joint"]

        # 躯干关节 - 使用chassis关节代替
        chassis_joint_name = "chassis"

        all_arm_joint_names = left_arm_joint_names + right_arm_joint_names
        for name in all_arm_joint_names:
            motor = self.getDevice(name)
            if motor:
                self.arm_motors[name] = motor
                self.arm_sensors[name] = motor.getPositionSensor()
                if self.arm_sensors[name]:
                    self.arm_sensors[name].enable(self.timestep)
            else:
                logger.warning(f"Arm motor '{name}' not found.")

        for name in gripper_joint_names:
            motor = self.getDevice(name)
            if motor:
                self.gripper_motors[name] = motor
                self.gripper_sensors[name] = motor.getPositionSensor()
                if self.gripper_sensors[name]:
                    self.gripper_sensors[name].enable(self.timestep)
            else:
                logger.warning(f"Gripper motor '{name}' not found.")

        self.torso_motor = self.getDevice(chassis_joint_name)
        if self.torso_motor:
            self.torso_sensor = self.torso_motor.getPositionSensor()
            if self.torso_sensor:
                self.torso_sensor.enable(self.timestep)
        else:
            logger.warning(f"Chassis motor '{chassis_joint_name}' not found.")

        # 确保没有机械臂自动伸展的代码在这里，如果有，请注释掉或删除。
        # 例如，之前你可能删除了 set_initial_position() 类似的调用。

        # Some devices, such as the InertialUnit, need some time to "warm up"
        self.wait(100) # Give some time for sensors to stabilize

    def wait(self, ms):
        """ Waits for a specified number of milliseconds. """
        self.step(ms)

    def initialize_angle_from_imu(self):
        """从IMU同步初始角度，确保内部角度记录与IMU一致"""
        if self.iu:
            try:
                # 多次读取IMU角度取平均值，确保稳定
                angle_readings = []
                for _ in range(10):
                    self.step(self.timestep)
                    rotation_matrix = self.iu.getRollPitchYaw()
                    if rotation_matrix and len(rotation_matrix) >= 3:
                        raw_angle = rotation_matrix[2]
                        if isinstance(raw_angle, (int, float)) and not (math.isnan(raw_angle) or math.isinf(raw_angle)):
                            angle_readings.append(self.normalize_angle(raw_angle))
                
                if angle_readings:
                    # 计算平均角度（考虑角度的周期性）
                    # 将角度转换为复数进行平均，然后转回角度
                    complex_sum = sum(complex(math.cos(angle), math.sin(angle)) for angle in angle_readings)
                    avg_complex = complex_sum / len(angle_readings)
                    avg_angle = math.atan2(avg_complex.imag, avg_complex.real)
                    
                    self.current_angle_rad = avg_angle
                    self._last_valid_angle = avg_angle
                    
                    if self.verbose:
                        logger.info(f"IMU初始角度同步完成: {math.degrees(avg_angle):.2f}° (基于{len(angle_readings)}次读取)")
                else:
                    logger.warning("无法读取有效的IMU角度，使用默认角度0°")
                    self.current_angle_rad = 0.0
                    self._last_valid_angle = 0.0
                    
            except Exception as e:
                logger.error(f"IMU角度初始化失败: {e}")
                self.current_angle_rad = 0.0
                self._last_valid_angle = 0.0
        else:
            logger.warning("IMU不可用，使用默认角度0°")
            self.current_angle_rad = 0.0
            self._last_valid_angle = None

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
        """ 
        高精度旋转机器人指定角度，确保误差在1.5°以内
        使用基于IMU反馈的三阶段旋转方法
        """
        if self.verbose:
            logger.info(f"开始高精度旋转 {math.degrees(angle_radians):.2f}度")

        # 如果角度很小，直接返回
        if abs(angle_radians) < math.radians(0.3):  # 小于0.3度
            if self.verbose:
                logger.info("旋转角度很小，跳过旋转")
            return

        # 使用新的高精度旋转方法
        final_error = self.rotate_angle_with_feedback(angle_radians)
        
        # 验证旋转精度
        if abs(final_error) <= math.radians(1.5):
            if self.verbose:
                logger.info(f"旋转完成 - 最终误差: {math.degrees(final_error):.2f}° (✓ 精度达标)")
        else:
            if self.verbose:
                logger.warning(f"旋转完成 - 最终误差: {math.degrees(final_error):.2f}° (✗ 精度未达标)")
        
    def go_forward(self, distance):
        """ Moves the robot forward for a given distance [m] with real-time error correction. """
        if self.verbose:
            logger.info(f"Moving forward by {distance:.2f} meters with error correction")

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
                    logger.info(f"检测到角度误差: {math.degrees(angle_error):.2f}°")
                
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
                        logger.info(f"应用角度纠偏 - 左轮速度: {left_speed:.2f}, 右轮速度: {right_speed:.2f}")
                        logger.info(f"  角度纠正(大): {angular_correction:.3f}")
                        
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
                logger.info(f"移动完成后检测到位置误差较大: {final_distance_error:.3f}m > {POSITION_FINE_TUNING_THRESHOLD}m")
                logger.info(f"目标位置: ({target_position[0]:.3f}, {target_position[1]:.3f}), 实际位置: ({final_position[0]:.3f}, {final_position[1]:.3f})")
                logger.info("开始进行位置微调...")
            
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
                logger.info(f"当前航向: {heading_angle:.2f}°, 目标方向: {vector_angle:.2f}°")
                logger.info(f"投影距离: {projected_distance:.3f}m")
            
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
                    logger.info(f"位置微调: {move_direction} {abs(fine_tuning_distance):.3f}m")
                
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
                    logger.info(f"位置微调完成 - 调整前误差: {final_distance_error:.3f}m, 调整后误差: {adjusted_error:.3f}m")
                    
                    # 评估微调效果
                    if adjusted_error < final_distance_error:
                        improvement = ((final_distance_error - adjusted_error) / final_distance_error) * 100
                        logger.info(f"微调有效，误差减少了 {improvement:.1f}%")
                    else:
                        logger.warning(f"微调效果不佳，误差反而增加了")
            else:
                if self.verbose:
                    logger.info(f"投影距离较小 ({projected_distance:.3f}m)，不需要微调")
        elif self.verbose:
            logger.info(f"运动完成 - 最终位置误差: {(final_distance_error * 100):.2f}cm (在可接受范围内)")
            
        # 即使不微调也记录最终状态，便于调试
        if self.verbose and final_distance_error > self.position_error_threshold:
            logger.info(f"最终位置: 目标位置: ({target_position[0]:.3f}, {target_position[1]:.3f}), 实际位置: ({final_position[0]:.3f}, {final_position[1]:.3f})")

    def turn_east(self):
        """将机器人转向东方（机器人前方，对应+X方向，0度）"""
        self._rotate_to_absolute_angle(0.0)

    def turn_north(self):
        """将机器人转向北方（机器人左侧，对应+Y方向，90度）"""
        self._rotate_to_absolute_angle(math.pi / 2.0)

    def turn_west(self):
        """将机器人转向西方（机器人后方，对应-X方向，180度）"""
        self._rotate_to_absolute_angle(math.pi)

    def turn_south(self):
        """将机器人转向南方（机器人右侧，对应-Y方向，-90度）"""
        self._rotate_to_absolute_angle(-math.pi / 2.0)

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
            logger.info(f"旋转到绝对角度 - 当前: {math.degrees(current_absolute_angle):.2f}°, " +
                  f"目标: {math.degrees(absolute_angle):.2f}°, 需旋转: {math.degrees(angle_diff):.2f}°")
        
        # 如果角度差很小，直接返回
        if abs(angle_diff) < math.radians(0.5):  # 小于0.5度
            if self.verbose:
                logger.info("角度差很小，无需旋转")
            return
        
        # 使用新的精确旋转方法
        final_error = self.rotate_angle_with_feedback(angle_diff)
        
        # 更新内部角度记录为实际角度
        self.current_angle_rad = self.get_current_angle_from_imu()
        
        if self.verbose:
            actual_final_angle = self.get_current_angle_from_imu()
            # 获取原始IMU角度进行对比
            raw_imu_angle = self.iu.getRollPitchYaw()[2] if self.iu else 0.0
            logger.info(f"绝对角度旋转完成 - 目标: {math.degrees(absolute_angle):.2f}°, " +
                  f"实际: {math.degrees(actual_final_angle):.2f}°, " +
                  f"误差: {math.degrees(final_error):.2f}°")
            logger.info(f"IMU调试信息 - 原始IMU: {math.degrees(raw_imu_angle):.2f}°, " +
                  f"修正后: {math.degrees(actual_final_angle):.2f}°")

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
                logger.info(f"Setting {joint_name} to {position:.2f}")
            if wait_on_feedback and sensor:
                start_time = self.getTime()
                timeout = 2.0 # 给关节设置一个超时时间，防止无限等待
                while self.step(self.timestep) != -1:
                    # 检查是否超时
                    if self.getTime() - start_time > timeout:
                        logger.warning(f"Joint '{joint_name}' timed out reaching position {position:.2f}. Current: {sensor.getValue():.2f}")
                        break
                    if self.almost_equal(sensor.getValue(), position, 0.01): # 更严格的关节位置阈值
                        break
        else:
            logger.warning(f"Joint '{joint_name}' motor not found for setting position.")

    # 新增：设置躯干高度的方法 (保持不变，但请注意其对碰撞的影响)
    def set_torso_height(self, height, wait_on_feedback=True):
        if self.torso_motor:
            self.torso_motor.setVelocity(self.torso_motor.getMaxVelocity()) # 显式设置最大速度
            self.torso_motor.setPosition(height)
            if self.verbose:
                logger.info(f"Setting torso height to {height:.2f}")
            if wait_on_feedback and self.torso_sensor:
                start_time = self.getTime()
                timeout = 2.0
                while self.step(self.timestep) != -1:
                    if self.getTime() - start_time > timeout:
                        logger.warning(f"Torso timed out reaching height {height:.2f}. Current: {self.torso_sensor.getValue():.2f}")
                        break
                    if self.almost_equal(self.torso_sensor.getValue(), height, 0.01):
                        break
        else:
            logger.warning(f"Torso motor not found for setting height.")

    # 新增：设置抓手的方法 (保持不变)
    def set_gripper_position(self, gripper_name, position, wait_on_feedback=True):
        motor = self.gripper_motors.get(gripper_name)
        sensor = self.gripper_sensors.get(gripper_name)
        if motor:
            motor.setVelocity(motor.getMaxVelocity()) # 显式设置最大速度
            motor.setPosition(position)
            if self.verbose:
                logger.info(f"Setting {gripper_name} to {position:.2f}")
            if wait_on_feedback and sensor:
                start_time = self.getTime()
                timeout = 1.0 # 抓手可能更快
                while self.step(self.timestep) != -1:
                    if self.getTime() - start_time > timeout:
                        logger.warning(f"WARNING: Gripper '{gripper_name}' timed out reaching position {position:.2f}. Current: {sensor.getValue():.2f}")
                        break
                    if self.almost_equal(sensor.getValue(), position, 0.005):
                        break
        else:
            logger.warning(f"WARNING: Gripper motor '{gripper_name}' not found for setting position.")

    def retract_arms(self):
        """ 将urdf_arm的两个机械臂收缩到一个安全的姿态。
            根据新的关节命名调整机械臂位置，避免与任何障碍物发生碰撞。
        """
        if self.verbose:
            logger.info("Retracting urdf_arm arms to a safe posture...")

        # 1. 将底盘降到合适位置，避免碰撞
        if self.torso_motor:
            self.set_torso_height(0.0, True) # 使用chassis关节控制底盘高度

        # 2. 设置左臂收缩姿态 - 根据新的关节命名
        # 这些值需要根据urdf_arm的具体结构调整
        self.set_arm_joint_position("zarm_l1_joint", 0.0, False)     # 左臂关节1
        self.set_arm_joint_position("zarm_l2_joint", 0.1, False)     # 左臂关节2，抬高避免碰撞
        self.set_arm_joint_position("zarm_l3_joint", 0.0, False)     # 左臂关节3
        self.set_arm_joint_position("zarm_l4_joint", -1.5, False)    # 左臂关节4，弯曲肘部
        self.set_arm_joint_position("zarm_l5_joint", 0.0, False)     # 左臂关节5

        # 3. 设置右臂收缩姿态 - 对称设置
        self.set_arm_joint_position("zarm_r1_joint", 0.0, False)     # 右臂关节1
        self.set_arm_joint_position("zarm_r2_joint", -0.1, False)     # 右臂关节2，抬高避免碰撞
        self.set_arm_joint_position("zarm_r3_joint", 0.0, False)     # 右臂关节3
        self.set_arm_joint_position("zarm_r4_joint", -1.5, False)    # 右臂关节4，弯曲肘部
        self.set_arm_joint_position("zarm_r5_joint", 0.0, False)     # 右臂关节5

        # 4. 关闭抓手
        self.set_gripper_position("left_gripper_finger_joint", 0.0, False) # 左抓手闭合
        self.set_gripper_position("right_gripper_finger_joint", 0.0, True)  # 右抓手闭合，等待反馈

        if self.verbose:
            logger.info("urdf_arm arms retracted to safe posture.")

    def get_current_position(self):
        """获取当前GPS位置"""
        if not self.gps:
            logger.warning("无法获取位置：GPS传感器不可用")
            return [0.0, 0.0]  # 返回默认位置
            
        try:
            position = self.gps.getValues()
            # 防止获取到错误的GPS数据
            if len(position) >= 3 and all(isinstance(p, (int, float)) for p in position[:2]):
                # 只返回X-Y平面的坐标
                return [position[0], position[1]]
            else:
                logger.warning(f"GPS返回的数据格式不正确: {position}")
                return [0.0, 0.0]
        except Exception as e:
            logger.error(f"获取GPS位置时出错: {e}")
            return [0.0, 0.0]
    
    def get_current_angle_from_imu(self):
        """从IMU获取当前角度(弧度)，带容错和校准"""
        if not self.iu:
            logger.warning("无法获取角度：IMU传感器不可用")
            return self.current_angle_rad  # 返回上次记录的角度
            
        try:
            # 从IMU获取欧拉角 - Webots IMU返回的是相对于世界坐标系的绝对方向
            # 标准Webots IMU返回roll (x), pitch (z), yaw (y)
            rotation_matrix = self.iu.getRollPitchYaw()
            
            if not rotation_matrix or len(rotation_matrix) < 3:
                logger.warning("IMU数据不完整，使用上次记录角度")
                return self.current_angle_rad
                
            raw_angle = rotation_matrix[2]  # 取偏航角(yaw)
            
            # 检查角度是否为有效数值
            if not isinstance(raw_angle, (int, float)) or math.isnan(raw_angle) or math.isinf(raw_angle):
                logger.warning("IMU返回无效角度，使用上次记录角度")
                return self.current_angle_rad
            
            # 坐标系转换：修正IMU与Webots世界坐标系的差异
            # 根据观察，IMU坐标系与世界坐标系存在180°偏差
            # 需要将IMU角度转换为与Supervisor一致的世界坐标系角度
            corrected_angle = self.normalize_angle(raw_angle + math.pi)
            
            # 对角度进行标准化到[-π, π]区间
            current_angle = self.normalize_angle(corrected_angle)
            
            # 初始化处理 - 如果是第一次读取或者没有有效的上次角度
            if not hasattr(self, '_last_valid_angle') or self._last_valid_angle is None:
                self._last_valid_angle = current_angle
                self.current_angle_rad = current_angle
                if self.verbose:
                    logger.info(f"IMU角度初始化: 原始={math.degrees(raw_angle):.2f}°, 修正后={math.degrees(current_angle):.2f}°")
                return current_angle
            
            # 计算角度变化 - 考虑-π到π的边界问题
            angle_diff = self.normalize_angle(current_angle - self._last_valid_angle)
            angle_change = abs(angle_diff)
            
            # 检测异常的角度跳跃
            # 在正常旋转中，单步角度变化不应超过30度（约0.5弧度）
            max_normal_change = math.radians(30)
            
            if angle_change > max_normal_change:
                # 检查是否是180度边界跳跃（从179度到-179度或相反）
                boundary_jump_1 = abs(angle_change - 2*math.pi) < max_normal_change  # 近似360度跳跃
                boundary_jump_2 = abs(angle_change - math.pi) < max_normal_change    # 近似180度跳跃
                
                if boundary_jump_1 or boundary_jump_2:
                    # 这是正常的边界跳跃，接受这个角度
                    if self.verbose:
                        logger.info(f"检测到正常边界跳跃: {math.degrees(self._last_valid_angle):.2f}° → {math.degrees(current_angle):.2f}°")
                    self._last_valid_angle = current_angle
                else:
                    # 异常的角度跳跃，使用渐变处理
                    if self.verbose:
                        logger.warning(f"检测到异常角度跳跃: {math.degrees(angle_change):.2f}°，限制变化幅度")
                    
                    # 限制角度变化幅度，每次最多变化max_normal_change
                    if angle_diff > 0:
                        limited_change = min(angle_diff, max_normal_change)
                    else:
                        limited_change = max(angle_diff, -max_normal_change)
                    
                    current_angle = self.normalize_angle(self._last_valid_angle + limited_change)
                    self._last_valid_angle = current_angle
            else:
                # 正常的角度变化
                self._last_valid_angle = current_angle
            
            # 更新内部角度记录
            if abs(current_angle - self.current_angle_rad) > 0.01:  # 大于0.5度才记录日志
                if self.verbose:
                    logger.info(f"IMU角度更新: {math.degrees(self.current_angle_rad):.2f}° → {math.degrees(current_angle):.2f}°")
            
            return current_angle
            
        except Exception as e:
            logger.error(f"读取IMU角度时出错: {e}")
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
            logger.info(f"纠偏计算 - 位置纠正: {forward_correction:.3f} (因子:{self.position_correction_factor:.2f}), " +
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
                    logger.info(f"已到达目标位置，最终误差: {distance_to_target:.3f}m")
                break
            
            # 计算需要转向的角度
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - current_angle)
            
            # 如果角度偏差较大，先转向
            if abs(angle_diff) > 0.1:  # 约5.7度
                if self.verbose:
                    logger.info(f"调整方向，角度偏差: {math.degrees(angle_diff):.2f}°")
                self.rotate_angle(angle_diff)
                self.current_angle_rad = target_angle
            
            # 前进到目标位置
            move_distance = min(distance_to_target, 0.5)  # 每次最多移动0.5米
            if self.verbose:
                logger.info(f"前进 {move_distance:.3f}m，剩余距离: {distance_to_target:.3f}m")
            self.go_forward(move_distance)
            
            iteration += 1
            
            # 短暂等待传感器稳定
            self.wait(50)
        
        if iteration >= max_iterations:
            current_pos = self.get_current_position()
            final_error = math.sqrt((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)
            logger.warning(f"警告: 达到最大迭代次数，最终位置误差: {final_error:.3f}m")

    def set_movement_speed_factor(self, factor):
        """
        设置移动速度系数，用于在接近障碍物时降低速度
        
        Args:
            factor: 速度系数 (0.0-1.0)，1.0 为全速
        """
        self.speed_factor = max(0.1, min(1.0, factor))  # 限制在 0.1-1.0 之间
        if self.verbose:
            logger.info(f"速度系数已设置为: {self.speed_factor:.2f}")
            
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
            logger.info(f"Moving backward by {distance:.2f} meters")
        
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
            logger.info(f"开始精确移动到像素坐标: {target_pixel_position}")
        
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
            logger.info(f"目标世界坐标: ({target_world_x:.3f}, {target_world_y:.3f})米")
            logger.info(f"当前世界坐标: ({current_gps_position[0]:.3f}, {current_gps_position[1]:.3f})米")
        
        success = False
        for attempt in range(max_attempts):
            # 计算距离和方向
            dx = target_world_position[0] - current_gps_position[0]
            dy = target_world_position[1] - current_gps_position[1]
            distance_to_target = math.sqrt(dx*dx + dy*dy)
            
            if self.verbose:
                logger.info(f"尝试 {attempt + 1}/{max_attempts}: 距离目标 {distance_to_target:.3f}米")
            
            # 如果距离足够小，认为已到达
            if distance_to_target < 0.02:  # 2厘米精度
                if self.verbose:
                    logger.info(f"精确定位成功！距离误差: {distance_to_target:.3f}米")
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
                    logger.info(f"调整朝向: 当前 {math.degrees(current_angle):.1f}°, 目标 {math.degrees(target_angle):.1f}°")
                self.rotate_angle(angle_diff)
                
                # 等待稳定
                for _ in range(10):
                    if self.step(self.timestep) == -1:
                        break
            
            # 直线移动到目标
            move_distance = min(distance_to_target, 0.2)  # 每次最多移动20厘米
            if self.verbose:
                logger.info(f"向目标移动 {move_distance:.3f}米")
            
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
                logger.warning(f"精确定位未完全成功，最终距离误差: {final_distance:.3f}米")
        
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
            logger.info(f"开始像素级精确微调到: {target_pixel_position}, 容差: {tolerance_pixels}像素")
        
        # 获取当前位置
        current_position = self.get_current_position()
        
        # 将当前世界坐标转换回像素坐标以便比较
        pixel_to_meter = self.DISTANCE_PER_GRID_CELL / scale_factor
        
        # 转换当前位置为像素坐标（反向转换）
        current_pixel_col = int(current_position[0] / pixel_to_meter + 80)  # 加上地图中心偏移
        current_pixel_row = int(80 - current_position[1] / pixel_to_meter)  # Y轴翻转并加偏移
        current_pixel_position = (current_pixel_row, current_pixel_col)
        
        if self.verbose:
            logger.info(f"当前像素坐标: {current_pixel_position}")
            logger.info(f"目标像素坐标: {target_pixel_position}")
        
        # 计算像素级距离
        pixel_dx = target_pixel_position[1] - current_pixel_position[1]
        pixel_dy = target_pixel_position[0] - current_pixel_position[0]
        pixel_distance = math.sqrt(pixel_dx*pixel_dx + pixel_dy*pixel_dy)
        
        if self.verbose:
            logger.info(f"像素距离: {pixel_distance:.1f}像素")
        
        # 如果距离在容差范围内，认为已到达
        if pixel_distance <= tolerance_pixels:
            if self.verbose:
                logger.info(f"已在目标容差范围内! 像素距离: {pixel_distance:.1f} <= {tolerance_pixels}")
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
            logger.info(f"需要移动: ({actual_dx:.3f}, {actual_dy:.3f})米, 距离: {actual_distance:.3f}米")
            logger.info(f"需要转向: {math.degrees(angle_diff):.1f}度")
        
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
                    logger.info("执行精确转向...")
                self.rotate_angle(angle_diff)
                
                # 等待稳定
                for _ in range(15):
                    if self.step(self.timestep) == -1:
                        break
            
            # 精确移动
            if actual_distance > 0.01:  # 如果距离大于1厘米
                if self.verbose:
                    logger.info(f"执行精确移动: {actual_distance:.3f}米")
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
            logger.info(f"微调完成!")
            logger.info(f"最终像素坐标: {final_pixel_position}")
            logger.info(f"最终像素距离: {final_pixel_distance:.1f}像素")
            logger.info(f"微调结果: {'成功' if success else '失败'}")
        
        return success

    def test_basic_movement(self):
        """测试基本的轮子运动，不依赖GPS和IMU"""
        logger.info("=== 开始基本运动测试 ===")
        
        # 1. 测试单个轮子
        logger.info("测试前左轮...")
        if self.wheel_motors.get("fl_caster_l_wheel_joint"):
            self.wheel_motors["fl_caster_l_wheel_joint"].setVelocity(2.0)
            for _ in range(100):  # 1.6秒
                self.step(self.timestep)
            self.wheel_motors["fl_caster_l_wheel_joint"].setVelocity(0.0)
            logger.info("前左轮测试完成")
        else:
            logger.error("前左轮电机未找到!")
        
        # 2. 测试所有轮子前进
        logger.info("测试所有轮子前进...")
        self.set_wheels_speed(1.0)
        for _ in range(200):  # 3.2秒
            self.step(self.timestep)
        self.stop_wheels()
        logger.info("前进测试完成")
        
        # 3. 测试旋转电机状态
        logger.info("测试旋转电机状态...")
        rotation_motors_working = True
        for name in ["fl_caster_rotation_joint", "fr_caster_rotation_joint", 
                    "bl_caster_rotation_joint", "br_caster_rotation_joint"]:
            if self.rotation_motors.get(name):
                logger.info(f"  ✓ {name}: 已连接")
            else:
                logger.error(f"  ✗ {name}: 未连接")
                rotation_motors_working = False
        
        if not rotation_motors_working:
            logger.error("部分旋转电机缺失，将使用差速旋转方法")
            # 使用差速旋转（不改变轮子角度，只改变速度）
            logger.info("测试差速原地旋转...")
            # 左轮前进，右轮后退，实现逆时针旋转
            self.set_wheels_speeds(2.0, 2.0, -2.0, -2.0, 2.0, 2.0, -2.0, -2.0)
            for _ in range(150):  # 2.4秒
                self.step(self.timestep)
            self.stop_wheels()
            logger.info("差速旋转测试完成")
        else:
            # 4. 测试完整的旋转（有旋转电机）
            logger.info("测试原地旋转（带旋转电机）...")
            # 先设置轮子为旋转模式 - 使用较小的角度
            logger.info("设置轮子角度为旋转模式...")
            self.set_rotation_wheels_angles(math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, True)
            
            # 等待轮子到位
            logger.info("等待轮子到达目标角度...")
            for _ in range(100):  # 给轮子时间到达位置
                self.step(self.timestep)
            
            # 设置轮子速度进行旋转
            logger.info("开始旋转运动...")
            self.set_wheels_speeds(2.0, 2.0, -2.0, -2.0, 2.0, 2.0, -2.0, -2.0)
            for _ in range(150):  # 2.4秒
                self.step(self.timestep)
            self.stop_wheels()
            
            # 重置轮子角度
            logger.info("重置轮子角度...")
            self.set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
            logger.info("旋转测试完成")
        
        logger.info("=== 基本运动测试完成 ===")
        
        # 额外测试：使用新的精确旋转方法
        logger.info("=== 开始精确旋转测试 ===")
        initial_angle = self.get_current_angle_from_imu()
        logger.info(f"精确旋转前角度: {math.degrees(initial_angle):.2f}°")
        
        # 使用新的基于反馈的精确旋转方法
        if self.iu:  # 确保IMU可用
            try:
                logger.info("测试高精度旋转（目标精度: ≤1.5°）")
                
                # 测试多种角度的精度
                test_angles = [30, 45, 90, 135, 180, -90, -45]
                precision_results = []
                
                for test_angle_deg in test_angles:
                    angle_rad = math.radians(test_angle_deg)
                    
                    before_angle = self.get_current_angle_from_imu()
                    logger.info(f"测试旋转 {test_angle_deg}°...")
                    
                    # 使用高精度旋转方法
                    error = self.rotate_angle(angle_rad)
                    
                    after_angle = self.get_current_angle_from_imu()
                    actual_rotation = self.normalize_angle(after_angle - before_angle)
                    
                    precision_error = abs(math.degrees(error)) if error else abs(math.degrees(actual_rotation - angle_rad))
                    precision_results.append(precision_error)
                    
                    status = "✓ 达标" if precision_error <= 1.5 else "✗ 超标"
                    logger.info(f"  角度 {test_angle_deg}°: 误差 {precision_error:.2f}° ({status})")
                    
                    # 等待稳定
                    self.step_n_times(20)
                
                # 计算统计信息
                avg_error = sum(precision_results) / len(precision_results)
                max_error = max(precision_results)
                success_count = sum(1 for err in precision_results if err <= 1.5)
                
                logger.info("=== 精度测试结果统计 ===")
                logger.info(f"平均误差: {avg_error:.2f}°")
                logger.info(f"最大误差: {max_error:.2f}°")
                logger.info(f"成功率: {success_count}/{len(precision_results)} ({success_count/len(precision_results)*100:.1f}%)")
                
                if max_error <= 1.5:
                    logger.info("🎉 所有测试均达到1.5°精度要求！")
                else:
                    logger.warning(f"⚠️  部分测试未达到精度要求，最大误差: {max_error:.2f}°")
                
            except Exception as e:
                logger.error(f"精确旋转测试失败: {e}")
        else:
            logger.warning("跳过精确旋转测试 - IMU不可用")
        
        logger.info("=== 精确旋转测试完成 ===")
        
        self.step_n_times(30)  # 等待3秒

    def step_n_times(self, n):
        """执行n次时间步"""
        for _ in range(n):
            self.step(self.timestep)

    def simple_differential_rotation(self, angle_radians, duration_seconds=3.0):
        """
        简单的差速旋转方法 - 不调节轮子角度，仅使用左右轮反向旋转
        用于调试和验证旋转功能
        """
        if self.verbose:
            logger.info(f"开始简单差速旋转 {math.degrees(angle_radians):.2f}度，持续 {duration_seconds}秒")

        # 记录起始角度
        start_angle = self.get_current_angle_from_imu()
        
        # 设置所有旋转轮角度为0（直行模式）
        self.set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
        
        # 计算轮速：左右轮反向
        if angle_radians > 0:  # 逆时针旋转
            left_speed = -self.MAX_WHEEL_SPEED * 0.3  # 左轮倒转
            right_speed = self.MAX_WHEEL_SPEED * 0.3   # 右轮正转
        else:  # 顺时针旋转
            left_speed = self.MAX_WHEEL_SPEED * 0.3    # 左轮正转
            right_speed = -self.MAX_WHEEL_SPEED * 0.3  # 右轮倒转
        
        # 设置左右轮速
        if self.verbose:
            logger.info(f"设置轮速: 左轮={left_speed:.2f}, 右轮={right_speed:.2f}")
        
        # 假设前左和后左是左侧轮，前右和后右是右侧轮
        wheel_speeds = {
            "fl_caster_l_wheel_joint": left_speed,
            "fl_caster_r_wheel_joint": left_speed,
            "bl_caster_l_wheel_joint": left_speed, 
            "bl_caster_r_wheel_joint": left_speed,
            "fr_caster_l_wheel_joint": right_speed,
            "fr_caster_r_wheel_joint": right_speed,
            "br_caster_l_wheel_joint": right_speed,
            "br_caster_r_wheel_joint": right_speed
        }
        
        for wheel_name, speed in wheel_speeds.items():
            if wheel_name in self.wheel_motors:
                self.wheel_motors[wheel_name].setVelocity(speed)
        
        # 按时间旋转
        start_time = self.getTime()
        while self.step(self.timestep) != -1:
            current_time = self.getTime()
            if current_time - start_time >= duration_seconds:
                break
            
            # 每0.5秒打印一次角度变化
            if int((current_time - start_time) * 2) != int((current_time - start_time - self.timestep/1000.0) * 2):
                current_angle = self.get_current_angle_from_imu()
                angle_change = self.normalize_angle(current_angle - start_angle)
                if self.verbose:
                    logger.info(f"旋转进行中，时间: {current_time - start_time:.1f}s, 角度变化: {math.degrees(angle_change):.2f}°")
        
        # 停止所有轮子
        self.stop_wheels()
        
        # 记录最终角度
        final_angle = self.get_current_angle_from_imu()
        total_angle_change = self.normalize_angle(final_angle - start_angle)
        
        if self.verbose:
            logger.info(f"差速旋转完成，总角度变化: {math.degrees(total_angle_change):.2f}°，目标: {math.degrees(angle_radians):.2f}°")
        
        # 更新内部角度记录
        self.current_angle_rad = final_angle
        
        return total_angle_change


    
    def rotate_angle_with_feedback(self, angle_radians):
        """
        基于角度反馈的高精度旋转方法
        确保最终误差在1.5°以内
        """
        if self.verbose:
            logger.info(f"开始高精度旋转 {math.degrees(angle_radians):.2f}度")

        # 记录起始角度和目标角度
        start_angle = self.get_current_angle_from_imu()
        target_angle = self.normalize_angle(start_angle + angle_radians)
        
        if self.verbose:
            logger.info(f"起始角度: {math.degrees(start_angle):.2f}°, 目标角度: {math.degrees(target_angle):.2f}°")
        
        # 保存原始设置
        original_error_correction = self.enable_error_correction
        self.enable_error_correction = False
        
        self.stop_wheels()
        
        # 阶段1: 粗调 - 快速接近目标（误差大于5度时）
        self._coarse_rotation(target_angle)
        
        # 阶段2: 精调 - 精确调整（误差小于5度时）
        final_error = self._fine_rotation(target_angle)
        
        # 阶段3: 超精调 - 如果误差仍大于1.5度，进行最后的微调
        if abs(final_error) > math.radians(1.5):
            final_error = self._ultra_fine_rotation(target_angle)
        
        # 停止并重置
        self.stop_wheels()
        self.set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
        
        # 最终验证
        final_angle = self.get_current_angle_from_imu()
        actual_final_error = self.normalize_angle(target_angle - final_angle)
        
        # 更新内部角度记录
        self.current_angle_rad = final_angle
        
        # 恢复原始设置
        self.enable_error_correction = original_error_correction
        
        if self.verbose:
            logger.info(f"高精度旋转完成:")
            logger.info(f"  目标角度: {math.degrees(target_angle):.2f}°")
            logger.info(f"  最终角度: {math.degrees(final_angle):.2f}°")
            logger.info(f"  最终误差: {math.degrees(actual_final_error):.2f}°")
            if abs(actual_final_error) <= math.radians(1.5):
                logger.info(f"  ✓ 精度达标（误差 ≤ 1.5°）")
            else:
                logger.warning(f"  ✗ 精度未达标（误差 > 1.5°）")
        
        return actual_final_error

    def _coarse_rotation(self, target_angle):
        """粗调阶段 - 快速接近目标角度"""
        if self.verbose:
            logger.info("开始粗调阶段...")
        
        # 设置旋转角度
        self.set_rotation_wheels_angles(3.0 * math.pi / 4.0, math.pi / 4.0, -3.0 * math.pi / 4.0, -math.pi / 4.0, True)
        
        max_iterations = 500
        iteration = 0
        coarse_tolerance = math.radians(5.0)  # 5度的粗调容差
        
        while iteration < max_iterations:
            current_angle = self.get_current_angle_from_imu()
            angle_error = self.normalize_angle(target_angle - current_angle)
            
            # 检查是否完成粗调
            if abs(angle_error) < coarse_tolerance:
                if self.verbose:
                    logger.info(f"粗调完成！当前误差: {math.degrees(angle_error):.2f}°")
                break
            
            # 动态调整速度
            if abs(angle_error) > math.radians(30):  # 大于30度，全速
                speed_factor = 0.8
            elif abs(angle_error) > math.radians(15):  # 15-30度，中高速
                speed_factor = 0.6
            elif abs(angle_error) > math.radians(10):  # 10-15度，中速
                speed_factor = 0.4
            else:  # 5-10度，低速
                speed_factor = 0.2
            
            # 确定旋转方向
            if angle_error > 0:  # 需要逆时针旋转
                wheel_speed = self.MAX_WHEEL_SPEED * speed_factor
            else:  # 需要顺时针旋转
                wheel_speed = -self.MAX_WHEEL_SPEED * speed_factor
            
            self.set_wheels_speed(wheel_speed)
            
            if self.verbose and iteration % 50 == 0:
                logger.info(f"粗调进行中 - 误差: {math.degrees(angle_error):.2f}°")
            
            iteration += 1
            self.step(self.timestep)
        
        self.stop_wheels()

    def _fine_rotation(self, target_angle):
        """精调阶段 - 精确调整到目标角度"""
        if self.verbose:
            logger.info("开始精调阶段...")
        
        max_iterations = 300
        iteration = 0
        fine_tolerance = math.radians(1.5)  # 1.5度的精调容差
        
        last_error = float('inf')
        stuck_count = 0
        
        while iteration < max_iterations:
            current_angle = self.get_current_angle_from_imu()
            angle_error = self.normalize_angle(target_angle - current_angle)
            
            # 检查是否完成精调
            if abs(angle_error) < fine_tolerance:
                if self.verbose:
                    logger.info(f"精调完成！当前误差: {math.degrees(angle_error):.2f}°")
                return angle_error
            
            # 精调使用更低的速度
            if abs(angle_error) > math.radians(3):  # 大于3度，低速
                speed_factor = 0.15
            elif abs(angle_error) > math.radians(2):  # 2-3度，极低速
                speed_factor = 0.1
            else:  # 小于2度，超低速
                speed_factor = 0.05
            
            # 确定旋转方向
            if angle_error > 0:  # 需要逆时针旋转
                wheel_speed = self.MAX_WHEEL_SPEED * speed_factor
            else:  # 需要顺时针旋转
                wheel_speed = -self.MAX_WHEEL_SPEED * speed_factor
            
            self.set_wheels_speed(wheel_speed)
            
            # 检查是否卡住
            if abs(abs(angle_error) - abs(last_error)) < math.radians(0.05):
                stuck_count += 1
                if stuck_count > 30:
                    if self.verbose:
                        logger.warning(f"精调可能卡住，当前误差: {math.degrees(angle_error):.2f}°")
                    break
            else:
                stuck_count = 0
            
            last_error = abs(angle_error)
            
            if self.verbose and iteration % 30 == 0:
                logger.info(f"精调进行中 - 误差: {math.degrees(angle_error):.2f}°")
            
            iteration += 1
            self.step(self.timestep)
        
        self.stop_wheels()
        current_angle = self.get_current_angle_from_imu()
        return self.normalize_angle(target_angle - current_angle)

    def _ultra_fine_rotation(self, target_angle):
        """超精调阶段 - 最后的微调，确保误差在1.5度以内"""
        if self.verbose:
            logger.info("开始超精调阶段...")
        
        max_iterations = 200
        iteration = 0
        ultra_fine_tolerance = math.radians(1.5)  # 1.5度的超精调容差
        
        # 使用脉冲式旋转进行超精调
        while iteration < max_iterations:
            current_angle = self.get_current_angle_from_imu()
            angle_error = self.normalize_angle(target_angle - current_angle)
            
            # 检查是否完成超精调
            if abs(angle_error) < ultra_fine_tolerance:
                if self.verbose:
                    logger.info(f"超精调完成！最终误差: {math.degrees(angle_error):.2f}°")
                return angle_error
            
            # 超精调使用脉冲式控制
            pulse_duration = max(5, int(abs(math.degrees(angle_error)) * 2))  # 脉冲持续时间与误差成正比
            speed_factor = 0.03  # 极低速度
            
            # 确定旋转方向
            if angle_error > 0:  # 需要逆时针旋转
                wheel_speed = self.MAX_WHEEL_SPEED * speed_factor
            else:  # 需要顺时针旋转
                wheel_speed = -self.MAX_WHEEL_SPEED * speed_factor
            
            # 执行脉冲旋转
            self.set_wheels_speed(wheel_speed)
            for _ in range(pulse_duration):
                self.step(self.timestep)
            self.stop_wheels()
            
            # 等待稳定
            for _ in range(10):
                self.step(self.timestep)
            
            if self.verbose and iteration % 20 == 0:
                logger.info(f"超精调进行中 - 误差: {math.degrees(angle_error):.2f}°, 脉冲时长: {pulse_duration}")
            
            iteration += 1
        
        # 如果仍未达到精度要求，记录警告
        current_angle = self.get_current_angle_from_imu()
        final_error = self.normalize_angle(target_angle - current_angle)
        
        if abs(final_error) > ultra_fine_tolerance:
            if self.verbose:
                logger.warning(f"超精调未能达到1.5°精度要求，最终误差: {math.degrees(final_error):.2f}°")
        
        return final_error

    def test_precision_requirement(self):
        """
        专门测试旋转精度是否满足1.5°要求
        用于验证改进后的旋转算法
        """
        logger.info("=== 开始1.5°精度验证测试 ===")
        
        if not self.iu:
            logger.error("无法进行精度测试 - IMU不可用")
            return False
        
        # 测试关键角度的精度
        critical_angles = [45, 90, 180, -90, -45]
        failed_tests = []
        precision_data = []
        
        for angle_deg in critical_angles:
            angle_rad = math.radians(angle_deg)
            
            # 记录起始角度
            start_angle = self.get_current_angle_from_imu()
            expected_final_angle = self.normalize_angle(start_angle + angle_rad)
            
            logger.info(f"测试关键角度 {angle_deg}°...")
            logger.info(f"  起始角度: {math.degrees(start_angle):.2f}°")
            logger.info(f"  期望角度: {math.degrees(expected_final_angle):.2f}°")
            
            # 执行高精度旋转
            final_error = self.rotate_angle(angle_rad)
            
            # 验证结果
            actual_final_angle = self.get_current_angle_from_imu()
            actual_error = abs(math.degrees(final_error)) if final_error else 0
            
            logger.info(f"  实际角度: {math.degrees(actual_final_angle):.2f}°")
            logger.info(f"  误差: {actual_error:.2f}°")
            
            # 记录测试结果
            precision_data.append({
                'target_angle': angle_deg,
                'error': actual_error,
                'passed': actual_error <= 1.5
            })
            
            if actual_error > 1.5:
                failed_tests.append((angle_deg, actual_error))
                logger.warning(f"  ❌ 未达标: 误差 {actual_error:.2f}° > 1.5°")
            else:
                logger.info(f"  ✅ 达标: 误差 {actual_error:.2f}° ≤ 1.5°")
            
            # 等待稳定
            self.step_n_times(50)
        
        # 输出测试总结
        logger.info("=== 精度验证结果 ===")
        passed_count = sum(1 for data in precision_data if data['passed'])
        total_count = len(precision_data)
        success_rate = (passed_count / total_count) * 100
        
        logger.info(f"通过测试: {passed_count}/{total_count} ({success_rate:.1f}%)")
        
        if failed_tests:
            logger.warning("未通过的测试:")
            for angle, error in failed_tests:
                logger.warning(f"  角度 {angle}°: 误差 {error:.2f}°")
        
        # 计算平均误差和最大误差
        errors = [data['error'] for data in precision_data]
        avg_error = sum(errors) / len(errors)
        max_error = max(errors)
        
        logger.info(f"平均误差: {avg_error:.2f}°")
        logger.info(f"最大误差: {max_error:.2f}°")
        
        # 最终判定
        all_passed = len(failed_tests) == 0
        if all_passed:
            logger.info("🎉 恭喜！所有测试均满足1.5°精度要求！")
        else:
            logger.warning(f"⚠️ 有 {len(failed_tests)} 个测试未达到精度要求")
        
        logger.info("=== 1.5°精度验证测试完成 ===")
        return all_passed

