from controller import Robot, Motor, PositionSensor, Supervisor, VacuumGripper
import math
import time
import sys
from pathlib import Path
import numpy as np

# 获取上级目录的绝对路径
parent_dir = str(Path(__file__).resolve().parents[2])
sys.path.append(parent_dir)

# 导入模块
from new_ik.ik_solve import ArmIk

# 创建 Robot 实例
robot = Robot()

# 创建Supervisor实例，因为你的urdf_arm节点设置了supervisor TRUE
supervisor = Supervisor()

# 获取当前世界的仿真步长
timestep = int(robot.getBasicTimeStep())
vacuum = robot.getDevice("right_gripper_vacuum")

# --- 辅助函数 ---

def get_motors_and_sensors(names_list):
    """--- 获取电机和位置传感器实例 ---"""
    motors = []
    sensors = []
    for name in names_list:
        motor = robot.getDevice(name)
        if motor:
            motors.append(motor)
            sensor = robot.getDevice(name + "_sensor")
            if sensor:
                sensor.enable(timestep)
                sensors.append(sensor)
            else:
                print(f"Warning: No position sensor found for motor '{name}'.")
        else:
            print(f"Error: Motor '{name}' not found. Please check device names.")
    return motors, sensors

def get_actual_joint_positions():
    """获取右臂关节的当前实际位置（弧度）"""
    positions = []
    if not right_arm_sensors:
        print("错误: 未找到右臂关节位置传感器")
        return positions
    
    # 确保传感器数量与关节数量匹配
    if len(right_arm_sensors) != len(right_arm_joint_names):
        print(f"警告: 右臂关节位置传感器数量({len(right_arm_sensors)})与关节数量({len(right_arm_joint_names)})不匹配")
    
    # 读取每个关节传感器的值
    for i, sensor in enumerate(right_arm_sensors):
        position = sensor.getValue()
        positions.append(position)
        print(f"关节 {right_arm_joint_names[i]} 的实际位置: {position:.6f} 弧度")
    
    return positions

def set_arm_joint_positions_with_feedback(motors, sensors, targets, 
                                         tolerance=0.01, max_steps=1000):
    """带反馈的关节位置控制，确保达到目标位置"""
    if len(targets) != len(motors):
        print(f"错误: 目标位置数量({len(targets)})与电机数量({len(motors)})不匹配")
        return False
    
    print(f"设置关节到目标位置: {targets}")
    
    # 再执行闭环反馈控制
    success = move_with_feedback(motors, sensors, targets, tolerance, max_steps//2)
    
    # 打印最终位置
    if success:
        current = get_actual_joint_positions()
        print(f"成功到达目标位置，实际位置: {current}")
    return success

def move_with_feedback(motors, sensors, targets, tolerance=0.01, max_steps=1000):
    """带反馈的位置控制，直到达到目标或超时"""
    for step in range(max_steps):
        for i, motor in enumerate(motors):
            motor.setPosition(targets[i])
        robot.step(timestep)
        
        # 检查是否达到目标位置
        current = [s.getValue() for s in sensors]
        if all(abs(t - c) < tolerance for t, c in zip(targets, current)):
            print(f"成功到达目标位置，用时{step}步")
            return True
    
    print(f"超时未到达目标位置，当前位置: {current}")
    print(f"targets位置: {targets}")
    return False

def set_gripper_position(gripper_motors_list, positions=None, default_position=0.0):
    """
    设置夹爪开合位置 - 支持为每个关节单独设置角度
    
    参数:
    - gripper_motors_list: 夹爪电机列表
    - positions: 每个关节的目标角度列表（可选）
    - default_position: 未指定角度时的默认值
    """
    print("夹爪电机名称列表:", [m.getName() for m in gripper_motors_list])

    if not gripper_motors_list:
        print("警告: 未找到夹爪电机，无法控制夹爪")
        return
    
    # 处理位置参数
    if positions is None:
        # 使用默认角度（原逻辑）
        position_list = [default_position] * len(gripper_motors_list)
        print(f"设置{len(gripper_motors_list)}个夹爪关节到统一角度: {default_position}")
    else:
        # 确保位置列表长度与电机列表匹配
        if len(positions) != len(gripper_motors_list):
            print(f"警告: 位置参数数量({len(positions)})与夹爪电机数量({len(gripper_motors_list)})不匹配")
            # 使用默认角度填充或截断位置列表
            position_list = positions[:len(gripper_motors_list)] + [default_position] * (len(gripper_motors_list) - len(positions))
        else:
            position_list = positions
        print(f"设置夹爪关节角度: {[round(pos, 4) for pos in position_list]}")
    
    # 设置每个电机位置
    for i, motor in enumerate(gripper_motors_list):
        motor.setPosition(position_list[i])
    
    robot.step(MOVE_DURATION_STEPS)  # 夹爪动作更快，缩短等待时间
    
    # 夹爪动作完成后获取并打印实际位置
    if right_gripper_sensors:
        gripper_positions = [sensor.getValue() for sensor in right_gripper_sensors]
        print(f"右夹爪关节实际位置: {[round(pos, 6) for pos in gripper_positions]}")

def move_robot_linear(dx=0.0, dy=0.0, dz=0.0):
    """
    直接平移机器人（忽略动力学，瞬间移动）
    
    参数:
    - dx, dy, dz: 分别为x、y、z轴方向的平移量（米）
    """
    # 获取机器人节点
    robot_node = supervisor.getSelf()
    if not robot_node:
        print("错误: 无法获取机器人节点")
        return
    
    # 获取当前位置
    translation_field = robot_node.getField('translation')
    current_pos = translation_field.getSFVec3f()
    
    # 计算新位置
    new_pos = [
        current_pos[0] + dx,
        current_pos[1] + dy,
        current_pos[2] + dz
    ]
    
    # 设置新位置
    translation_field.setSFVec3f(new_pos)
    print(f"机器人位置已更新为: {new_pos}")


# --- 主控制序列 ---
def run_arm_sequence():
    print("\n--- 开始UrdfArm右臂抓取放置任务 ---")

    # 1. 移动右臂到初始姿态并打开右夹爪
    print("步骤1: 移动右臂到初始姿态并打开夹爪")
    set_arm_joint_positions_with_feedback(
        right_arm_motors, right_arm_sensors, initial_right_arm_pose,
        tolerance=0.02, max_steps=8
    )
    set_gripper_position(right_gripper_motors, OPEN_GRIPPER_POS)
    robot.step(MOVE_DURATION_STEPS)
    # time.sleep(1)

    # 2. 移动右臂到预抓取姿态
    print("步骤2.1: 移动右臂到预抓取姿态")
    set_arm_joint_positions_with_feedback(
        right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose,
        tolerance=0.02, max_steps=80
    )
    # time.sleep(1)

    # print("步骤2: 机器人向前移动")
    # move_robot_linear(dx=0.5, dy=0.0, dz=0.0)
    # robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
    # time.sleep(1)

    # print("步骤2.2: 移动右臂到预抓取姿态")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose_2,
    #     tolerance=0.02, max_steps=80
    # )
    # time.sleep(3)

    # 3. 移动右臂到抓取姿态
    print("步骤3: 移动右臂到抓取姿态（靠近物体）")
    set_arm_joint_positions_with_feedback(
        right_arm_motors, right_arm_sensors, grasp_right_arm_pose,
        tolerance=0.02, max_steps=80
    )
    # time.sleep(10)
    robot.step(7*MOVE_DURATION_STEPS)

    # 4. 夹紧物体
    print("步骤4: 关闭右夹爪抓取物体")
    vacuum.enablePresence(timestep)
    vacuum.turnOn()
    set_gripper_position(right_gripper_motors, CLOSE_GRIPPER_POS)
    # robot.step(MOVE_DURATION_STEPS)
    # time.sleep(1)

    # 5. 抬起物体（回到预抓取姿态）
    print("步骤5: 抬起物体（回到预抓取姿态）")
    set_arm_joint_positions_with_feedback(
        right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose,
        tolerance=0.02, max_steps=80
    )

    # # 7. 移动右臂到放置姿态
    # print("步骤6: 移动右臂到放置姿态（目标位置）")
    # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
    # robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
    # # time.sleep(1)

    # # 6. 移动右臂到放置前的预备姿态
    # print("步骤7: 移动右臂到放置前预备姿态")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_place_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )
    # # time.sleep(1)

    # # 8. 松开物体
    # print("步骤8: 打开右夹爪释放物体")
    # vacuum.enablePresence(timestep)
    # vacuum.turnOff()
    # set_gripper_position(right_gripper_motors, OPEN_GRIPPER_POS)
    # # robot.step(MOVE_DURATION_STEPS)
    # # time.sleep(1)
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, place_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )
    # print("\n--- 右臂抓取放置任务完成 ---")
    
    # # 9.
    # print("步骤9: 移动右臂到预抓取姿态")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )
    # time.sleep(1)


# --- 定义机械臂、夹爪和头部关节名称 ---
# 根据 Webots 控制台输出的实际设备名称进行定义

# 左臂关节 (7个自由度) - 主要控制对象
left_arm_joint_names = [
    "zarm_l1_joint",  
    "zarm_l2_joint",  
    "zarm_l3_joint", 
    "zarm_l4_joint",
    "zarm_l5_joint", 
    "left_intermediate_joint"
]

# 左夹爪关节 - 定义但在此脚本中不主动控制
left_gripper_joint_names = [
    "left_gripper_finger_joint",
    "left_gripper_left_inner_finger_joint",
    "left_gripper_left_inner_knuckle_joint",
    "left_gripper_right_inner_finger_joint",
    "left_gripper_right_inner_knuckle_joint",
    "left_gripper_right_outer_knuckle_joint",
]

# 右臂关节 (7个自由度) - 主要控制对象
right_arm_joint_names = [
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
    "zarm_r5_joint",
    "right_intermediate_joint"
]

# 右夹爪关节 - 主要控制对象
right_gripper_joint_names = [
    "right_gripper_finger_joint",
    "right_gripper_left_inner_finger_joint",
    "right_gripper_left_inner_knuckle_joint",
    "right_gripper_right_inner_finger_joint",
    "right_gripper_right_inner_knuckle_joint",
    "right_gripper_right_outer_knuckle_joint",
]


# 头部关节 - 定义但在此脚本中不主动控制
head_joint_names = [
    "head",
    "head_pitch_motor"
]

print("Initializing robot devices...")
left_arm_motors, left_arm_sensors = get_motors_and_sensors(left_arm_joint_names)
left_gripper_motors, left_gripper_sensors = get_motors_and_sensors(left_gripper_joint_names)
right_arm_motors, right_arm_sensors = get_motors_and_sensors(right_arm_joint_names)
right_gripper_motors, right_gripper_sensors = get_motors_and_sensors(right_gripper_joint_names)
head_motors, head_sensors = get_motors_and_sensors(head_joint_names)
print("Device initialization complete.")

# --- 硬编码关节目标位置 ---
# 所有角度单位为弧度，需根据实际仿真环境调试
initial_right_arm_pose = [0.0, 0.0, 0.0, 0.0,0.0,0.0]  # 初始垂直姿态
pre_grasp_right_arm_pose = [-1.97400184e+00,  8.07771850e-02 ,0 , 1.83056540e+00 , 6.98207363e-02, 0] #准备位置 xyz:[0.2374993  ,-0.33825069 , 0.22093117]     RPY:[-3.02456926, -0.00675474,  0.09522905]
pre_grasp_right_arm_pose_2 = [-1.97400184e+00,  8.07771850e-02 ,0 , 1.83056540e+00 , 6.98207363e-02, 1.57]
grasp_right_arm_pose = [-1.7, -4.55066382e-02 , 1.69054925e-01 , 1.47089804e+00  ,4.53473656e-02,  1.55]  # xyz[0.2374993  ,-0.33825069 , 0.10093117]
pre_place_right_arm_pose = [-2.3,  8.07771850e-02 ,0 , 1.83056540e+00 , 6.98207363e-02, 0]                # 放置前预备姿态 xyz[0.2374993  ,-0.38825069 , 0.20093117] 
place_right_arm_pose = [ 0.0, -0.8, 0.0, 0.0,0.0,0.0]                    # 放置姿态 xyz[0.24993  ,-0.38825069 , 0.13093117]

# 右夹爪控制值 - 需根据夹爪模型调试
OPEN_GRIPPER_POS = [-0.7,0.7,-0.7,-0.7,0.7,0.7]  # 夹爪完全打开位置
CLOSE_GRIPPER_POS = [0.3,-0.3,0.3,0.3,-0.3,-0.3]  # 夹爪夹紧位置

# 运动时间步长
MOVE_DURATION_STEPS = 500  # 每次动作等待的时间步



# 主控制循环
task_executed = False
while robot.step(timestep) != -1:
    if not task_executed:
        # 初始化双臂规划器
        planner = ArmIk(
            model_file="/home/luocl/Desktop/my_project/urdf/urdf_arm_mix/urdf/urdf_arm_mix.urdf"
        )
        # 创建机器人运动学控制器实例，bu启用可视化
        arm_ik = ArmIk(visualize=False)
        
        # 获取初始关节角度
        q0 = arm_ik.get_init_q()
        
        # 1.准备位置 
        # 定义目标位置
        l_hand_pose = np.array([-0.01749985,0.29927,-0.21073])  # [x, y, z] 单位m
        r_hand_pose = np.array([0.2374993  ,-0.33825069 , 0.22093117])
        
        # 求解双臂IK
        sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, np.array([3.14, 0, 0]),np.array([-3.02456926, -0.00675474,  0.09522905]))
        # print("sol_q:",sol_q)
        pre_grasp_right_arm_pose = sol_q[19:25]
        # print("pre_grasp_right_arm_pose:",pre_grasp_right_arm_pose)
        # pre_grasp_right_arm_pose_2 = pre_grasp_right_arm_pose
        # pre_grasp_right_arm_pose_2[5] = 1.5
        # print("pre_grasp_right_arm_pose_2:",pre_grasp_right_arm_pose_2)
        
        # 2.抓取位置
        r_hand_pose = np.array([0.2374993  ,-0.33825069 , 0.12093117])
        sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, np.array([3.14, 0, 0]),np.array([-3.02456926, -0.00675474,  0.09522905]))
        # print("sol_q:",sol_q)
        grasp_right_arm_pose = sol_q[19:25]
        grasp_right_arm_pose = [-1.7, -4.55066382e-02 , 1.69054925e-01 , 1.47089804e+00  ,4.53473656e-02,  1.55]        
        # print("grasp_right_arm_pose:",grasp_right_arm_pose)

        # 这里可以添加你的控制逻辑
        run_arm_sequence()
        task_executed = True

print("控制器退出")
