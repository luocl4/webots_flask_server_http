from utilities import MyCustomRobot
import json
import time
import requests
import math
import sys
import os
from controller import (
    Robot,
    Motor,
    PositionSensor,
    Supervisor,
    VacuumGripper,
    TouchSensor,
)
from pathlib import Path
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
parent_dir = str(Path(__file__).resolve().parents[1])
sys.path.append(parent_dir)

from logger import logger
from new_ik.ik_solve import ArmIk

# --------------------------------------------------------------移动-------------------------------------------------

WEB_SERVER_URL = "http://127.0.0.1:5000/robot_command"
SCALE_FACTOR = 20
ROBOT_WIDTH = 16  # 机器人宽度,15.4,安全起见取17
ROBOT_HEIGHT = 15  # 机器人高度，14.2，安全起见取16

FINAL_APPROACH_DISTANCE = 4  # 接近目标的距离阈值(格子数)
original_start = (-1.4, 0.9)  # 原始地图中的起始点
# safety_radius = 1  # 安全膨胀半径，单位为精细化地图中的像素/格子 (允许小数点值)
goal = None
goal_yaw = None  # 添加目标偏航角变量，角度制，范围-180到180度

robot = MyCustomRobot(verbose=True)  # 可以将 verbose 设置为 True，方便调试
robot.initialize_devices()
timestep = int(robot.getBasicTimeStep())


class Map_transformer:
    def __init__(self, map_size=[9, 5.4], scale_factor: int = 20):
        self.map_size = map_size
        self.scale_factor = scale_factor
        logger.info(f"初始化坐标转换器: 地图尺寸={map_size}, 缩放因子={scale_factor}")

    def webots_to_map(self, webots_pos):
        x, z = webots_pos[0], webots_pos[1]  # 假设只提供了平面坐标[x, z]
        row = ((self.map_size[1] / 2) - z) * self.scale_factor
        col = (x + (self.map_size[0] / 2)) * self.scale_factor
        logger.debug(f"Webots坐标 [{x}, {z}] 转换为地图坐标: ({row}, {col})")
        return (row, col)

    def map_to_webots(self, map_pos):
        row, col = map_pos
        # 从地图坐标转回Webots坐标
        x = float(col) / self.scale_factor - (self.map_size[0] / 2)
        z = (self.map_size[1] / 2) - float(row) / self.scale_factor

        logger.debug(f"地图坐标 ({row}, {col}) 转换为Webots坐标: [{x}, {z}]")
        return [x, z]


# 检查关键传感器是否可用
if not hasattr(robot, 'gps') or not robot.gps:
    logger.warning("警告: GPS传感器未找到，机器人移动功能可能无法正常工作")

if not hasattr(robot, 'iu') or not robot.iu:
    logger.warning("警告: IMU传感器未找到，机器人转向功能可能无法正常工作")


# 从文件读取原始世界地图
def load_map_from_file(file_path):
    """
    从文本文件中加载地图数据

    Args:
        file_path: 地图文件路径

    Returns:
        二维数组表示的地图，0表示空地，1表示障碍物
    """
    try:
        world_map = []
        with open(file_path, 'r') as f:
            for line in f:
                # 去除行尾换行符，按空格分割，转换为浮点数，然后转为整数
                row = [float(cell) for cell in line.strip().split()]
                if row:  # 确保行不为空
                    world_map.append(row)

        if not world_map:
            logger.warning(f"从文件 {file_path} 读取的地图为空，使用默认地图")
            return None

        logger.info(
            f"成功从文件 {file_path} 读取地图，大小: {len(world_map)}x{len(world_map[0])}"
        )
        return world_map
    except Exception as e:
        logger.error(f"读取地图文件 {file_path} 出错: {e}，使用默认地图")
        return None


# 尝试从文件加载地图
map_file_path = os.path.join(os.path.dirname(__file__), 'obstacle_map.txt')
refined_world_map = load_map_from_file(map_file_path)
map_transfor = Map_transformer()
start = map_transfor.webots_to_map(original_start)  # 将原始起始点转换为地图坐标
current_robot_position = start

# robot.retract_arms() # 机械臂在机器人开始规划和移动前收缩


def is_robot_position_valid(position, map_data=None):
    """
    检查考虑机器人大小后的位置是否有效
    机器人被视为中心在position，尺寸为ROBOT_WIDTH x ROBOT_HEIGHT的矩形
    优化：只检查机器人轮廓的四条边是否有障碍物，而不是整个边界框区域
    """
    map_data = refined_world_map if map_data is None else map_data

    # 计算机器人的边界框
    half_width = ROBOT_WIDTH / 2
    half_height = ROBOT_HEIGHT / 2

    # 确保位置坐标是整数
    position = (int(position[0]), int(position[1]))

    # 机器人的边界框顶点
    top_left = (position[0] - half_height, position[1] - half_width)
    bottom_right = (position[0] + half_height, position[1] + half_width)

    # 地图尺寸
    map_height = len(map_data)
    map_width = len(map_data[0]) if map_height > 0 else 0

    # 计算用于检查的边界坐标（取整以确保覆盖所有格子）
    top_row = math.floor(top_left[0])
    bottom_row = math.ceil(bottom_right[0])
    left_col = math.floor(top_left[1])
    right_col = math.ceil(bottom_right[1])

    # 1. 检查边界是否超出地图范围
    if (
        top_row < 0
        or bottom_row >= map_height
        or left_col < 0
        or right_col >= map_width
    ):
        return False

    # 2. 只检查机器人的四条边界线
    # 检查上下边界
    for col in range(left_col, right_col + 1):
        if map_data[top_row][col] != 0:
            return False
        if map_data[bottom_row][col] != 0:
            return False

    # 检查左右边界
    for row in range(top_row + 1, bottom_row):  # +1避免重复检查角点
        if map_data[row][left_col] != 0:
            return False
        if map_data[row][right_col] != 0:
            return False

    return True


def get_neighbors(map_data, position, previous=None, current_direction=None):
    """
    获取邻居节点，考虑直线优先策略

    Args:
        map_data: 地图数据
        position: 当前位置 (row, col)
        previous: 前一个位置 (row, col)，可为None
        current_direction: 当前移动方向，可为None

    Returns:
        邻居节点列表，优先返回与当前方向一致的节点
    """
    # 定义四个方向 (delta_row, delta_col, direction_name)
    directions = [
        (-1, 0, "north"),  # 北
        (0, -1, "west"),  # 西
        (0, 1, "east"),  # 东
        (1, 0, "south"),  # 南
    ]

    # 如果有前一个点，计算当前方向
    if previous is not None and current_direction is None:
        delta_row = position[0] - previous[0]
        delta_col = position[1] - previous[1]

        if delta_row < 0:
            current_direction = "north"
        elif delta_row > 0:
            current_direction = "south"
        elif delta_col < 0:
            current_direction = "west"
        elif delta_col > 0:
            current_direction = "east"

    # 生成邻居节点
    neighbors = []
    straight_neighbors = []  # 用于存储与当前方向一致的邻居

    for delta_row, delta_col, direction in directions:
        neighbor = (int(position[0] + delta_row), int(position[1] + delta_col))

        # 检查该位置是否有效
        if is_robot_position_valid(neighbor, map_data):
            # 如果与当前方向一致，添加到直线邻居列表
            if current_direction == direction:
                straight_neighbors.append(neighbor)
            else:
                neighbors.append(neighbor)

    # 优先返回直线邻居，然后是其他邻居
    return straight_neighbors + neighbors


def move_from_to_multi_steps(current, steps, direction):
    """
    执行多步移动：先转向指定方向，然后前进指定步数
    添加自适应速度控制，靠近障碍物时减速
    """
    if direction is None or steps <= 0:
        return

    distance_per_grid_cell = robot.DISTANCE_PER_GRID_CELL
    total_distance = distance_per_grid_cell * steps

    # 计算到最近障碍物的距离
    obstacle_distance = get_obstacle_distance(current, refined_world_map)

    # 安全距离参数
    safety_multiplier = min(1.0, obstacle_distance / 10.0)  # 障碍物距离小于10时开始减速
    safety_multiplier = max(0.15, safety_multiplier)  # 最小速度不低于15%

    print(f"===== 移动开始 =====")
    print(f"从 {current} 移动 {steps} 步 {direction}，总距离: {total_distance:.3f}米")
    print(f"距离最近障碍物: {obstacle_distance} 格, 速度系数: {safety_multiplier:.2f}")

    # 如果目标已设置，计算到目标的距离
    if goal:
        distance_to_goal = math.sqrt(
            (current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2
        )
        real_distance_meters = (
            distance_to_goal / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
        )
        # 使用logger记录移动前与目标点的距离
        logger.info(
            f"移动前: 从 {current} 移动 {steps} 步 {direction}，与目标点 {goal} 之间的欧氏距离: {distance_to_goal:.2f} 像素，实际距离: {real_distance_meters:.3f} 米"
        )

    # 转向
    if direction == "north":
        robot.turn_north()
    elif direction == "south":
        robot.turn_south()
    elif direction == "west":
        robot.turn_west()
    elif direction == "east":
        robot.turn_east()

    # 判断是否接近目标点 - 如果步数很小，说明接近目标点
    approaching_goal = steps <= FINAL_APPROACH_DISTANCE  # 使用常量作为接近目标的阈值

    if approaching_goal:
        original_error_correction = robot.enable_error_correction
        robot.enable_error_correction = False
        logger.info(
            f"接近目标点 (剩余步数: {steps} <= {FINAL_APPROACH_DISTANCE})，关闭纠偏功能以提高稳定性"
        )

    # 前进多步，使用自适应速度
    robot.set_movement_speed_factor(safety_multiplier)  # 设置速度系数
    robot.go_forward(total_distance)

    # 恢复默认设置
    robot.set_movement_speed_factor(1.0)  # 恢复默认速度

    if approaching_goal:
        # 恢复原始纠偏设置
        robot.enable_error_correction = original_error_correction
        logger.info(
            "恢复纠偏设置: " + ("开启" if original_error_correction else "关闭")
        )

    # 获取当前GPS位置计算移动后与目标的距离
    if goal:
        # 获取移动后的位置
        try:
            gps_position = robot.get_current_position()
            current_position = map_transfor.webots_to_map(
                gps_position
            )  # 转换为地图坐标
            distance_to_goal = math.sqrt(
                (current_position[0] - goal[0]) ** 2
                + (current_position[1] - goal[1]) ** 2
            )
            real_distance_meters = (
                distance_to_goal / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
            )
            # 使用logger记录移动后与目标点的距离
            logger.info(
                f"移动后: 当前位置 {current_position}，与目标点 {goal} 之间的欧氏距离: {distance_to_goal:.2f} 像素，实际距离: {real_distance_meters:.3f} 米"
            )
        except Exception as e:
            logger.error(f"移动后获取位置出错: {e}")


def send_robot_status(current_pos, status_message, object_id=None):
    """
    向服务器发送机器人状态信息

    Args:
        current_pos: 当前位置坐标 (row, col)
        status_message: 状态信息，可以是字符串或字典
        object_id: 可选的机器人对象ID，如果未指定则使用默认值
    """
    # 如果状态信息是字符串，则转换为字典格式
    payload = {
        "robot_position": current_pos,
        "status": status_message,
        "object_id": 1 if object_id is None else object_id,  # 使用提供的ID或默认值
    }

    try:
        requests.post("http://127.0.0.1:5000/robot_status", json=payload)
    except requests.exceptions.ConnectionError:
        logger.warning("Could not connect to web server. Is it running?")
    except Exception as e:
        logger.error(f"Error sending status: {e}")


def get_obstacle_distance(position, map_data, max_scan_distance=20):
    """
    计算从当前位置到最近障碍物的距离

    Args:
        position: 当前位置 (row, col)
        map_data: 地图数据
        max_scan_distance: 最大扫描距离

    Returns:
        最近障碍物距离，如果没找到则返回max_scan_distance
    """
    # 确保位置是整数坐标
    position = (int(position[0]), int(position[1]))

    directions = [
        (-1, 0),  # 北
        (1, 0),  # 南
        (0, -1),  # 西
        (0, 1),  # 东
        (-1, -1),  # 西北
        (-1, 1),  # 东北
        (1, -1),  # 西南
        (1, 1),  # 东南
    ]

    map_height = len(map_data)
    map_width = len(map_data[0]) if map_height > 0 else 0

    min_distance = max_scan_distance

    for dir_row, dir_col in directions:
        for distance in range(1, max_scan_distance + 1):
            scan_row = int(position[0] + dir_row * distance)
            scan_col = int(position[1] + dir_col * distance)

            # 检查是否超出地图边界
            if (
                scan_row < 0
                or scan_row >= map_height
                or scan_col < 0
                or scan_col >= map_width
            ):
                min_distance = min(min_distance, distance)
                break

            # 检查是否遇到障碍物
            if map_data[scan_row][scan_col] != 0:
                min_distance = min(min_distance, distance)
                break

    return min_distance


def BFS_planning(map_data, start, goal, object_id=None):
    """
    使用广度优先搜索(BFS)进行路径规划

    Args:
        map_data: 地图数据
        start: 起始位置 (row, col)
        goal: 目标位置 (row, col)
        object_id: 可选的机器人对象ID

    Returns:
        path: 简化后的路径 [((start_row, start_col), steps, direction), ...]
    """
    # 检查起点和终点是否有效
    if not is_robot_position_valid(start, map_data):
        error_message = f"起点 {start} 位置对机器人大小无效!"
        logger.warning(error_message)
        # 立即向后端发送状态信息，通知路径规划失败
        error_info = {
            "status": "error",
            "message": error_message,
            "current_position": start,
            "is_reachable": False,
            "error_message": error_message,
            "path_planning_complete": True,  # 明确标记路径规划已完成，但失败了
        }
        send_robot_status(start, error_info, object_id)
        return []

    if not is_robot_position_valid(goal, map_data):
        error_message = f"终点 {goal} 位置对机器人大小无效!"
        logger.warning(error_message)
        # 立即向后端发送状态信息，通知路径规划失败
        error_info = {
            "status": "error",
            "message": error_message,
            "current_position": start,
            "is_reachable": False,
            "error_message": error_message,
            "path_planning_complete": True,  # 明确标记路径规划已完成，但失败了
        }
        send_robot_status(start, error_info, object_id)
        return []

    # 使用队列实现BFS
    from collections import deque

    queue = deque([(start, None, None)])  # (position, parent, direction)

    # 记录已访问的节点
    visited = {start: None}

    nodes_explored = 0  # 统计探索的节点数

    while queue:
        current, parent, current_direction = queue.popleft()
        nodes_explored += 1

        # 到达目标
        if current == goal:
            logger.info(f"BFS算法完成: 探索了 {nodes_explored} 个节点")
            break

        # 探索邻居节点
        for neighbor in get_neighbors(map_data, current, parent, current_direction):
            if neighbor not in visited:
                # 计算新的移动方向
                delta_row = neighbor[0] - current[0]
                delta_col = neighbor[1] - current[1]

                if delta_row < 0:
                    new_direction = "north"
                elif delta_row > 0:
                    new_direction = "south"
                elif delta_col < 0:
                    new_direction = "west"
                elif delta_col > 0:
                    new_direction = "east"
                else:
                    new_direction = current_direction  # 保持当前方向

                queue.append((neighbor, current, new_direction))
                visited[neighbor] = (current, new_direction)

    if goal not in visited:
        error_message = f"路径规划: 从 {start} 到 {goal} 无路径可达!"
        print(error_message)
        # 向后端发送状态信息，通知路径规划失败
        error_info = {
            "status": "error",
            "message": error_message,
            "current_position": start,
            "is_reachable": False,
            "error_message": error_message,
            "path_planning_complete": True,  # 明确标记路径规划已完成，但失败了
        }
        send_robot_status(start, error_info, object_id)
        return []

    # 重构原始路径
    raw_path = []
    direction_path = []
    current = goal

    while current is not None:
        raw_path.append(current)
        if current == start:
            direction_path.append(None)
            break
        parent, direction = visited[current]
        direction_path.append(direction)
        current = parent

    raw_path.reverse()
    direction_path.reverse()

    logger.info(f"原始路径长度: {len(raw_path)} 步")

    path = []
    i = 0

    while i < len(raw_path) - 1:
        start_pos = raw_path[i]
        current_direction = (
            direction_path[i + 1] if i + 1 < len(direction_path) else None
        )

        if current_direction is None:
            i += 1
            continue

        # 计算连续相同方向的步数
        steps = 0
        j = i

        while j < len(raw_path) - 1 and j + 1 < len(direction_path):
            if direction_path[j + 1] == current_direction:
                steps += 1
                j += 1
            else:
                break

        distance_in_cells = steps

        if distance_in_cells < SCALE_FACTOR:
            simplified_steps = distance_in_cells / SCALE_FACTOR
            if simplified_steps < 0.1:  # 小于0.1个网格单位的移动
                simplified_steps = 0.1
        else:
            simplified_steps = (
                distance_in_cells / SCALE_FACTOR
            )  # 使用精确除法而不是四舍五入
            if simplified_steps > 10:
                max_steps_per_segment = 8
                segments = []
                remaining_steps = simplified_steps
                current_segment_pos = start_pos

                while remaining_steps > 0:
                    segment_steps = min(remaining_steps, max_steps_per_segment)
                    segments.append(
                        (current_segment_pos, segment_steps, current_direction)
                    )
                    remaining_steps -= segment_steps

                    # 更新下一段的起始位置 (估算，实际移动时会有所不同)
                    if current_direction == "north":
                        current_segment_pos = (
                            current_segment_pos[0] - segment_steps * SCALE_FACTOR,
                            current_segment_pos[1],
                        )
                    elif current_direction == "south":
                        current_segment_pos = (
                            current_segment_pos[0] + segment_steps * SCALE_FACTOR,
                            current_segment_pos[1],
                        )
                    elif current_direction == "west":
                        current_segment_pos = (
                            current_segment_pos[0],
                            current_segment_pos[1] - segment_steps * SCALE_FACTOR,
                        )
                    elif current_direction == "east":
                        current_segment_pos = (
                            current_segment_pos[0],
                            current_segment_pos[1] + segment_steps * SCALE_FACTOR,
                        )

                path.extend(segments)
                i = j  # 移动到下一个不同方向的起始点
                continue

        # 添加到简化路径
        path.append((start_pos, simplified_steps, current_direction))
        i = j  # 移动到下一个不同方向的起始点

    logger.info(f"路径规划: 原始路径 {len(raw_path)} 步 → 简化路径 {len(path)} 步")
    return path


def move_to_precise_pixel_position(
    current_pixel_pos, target_pixel_pos, scale_factor=SCALE_FACTOR
):
    """
    精确移动到指定的像素位置，支持小数步长

    Args:
        current_pixel_pos: 当前像素位置 (row, col)
        target_pixel_pos: 目标像素位置 (row, col)
        scale_factor: 缩放因子

    Returns:
        bool: 是否成功移动
    """
    # 计算像素级的移动向量
    pixel_dx = target_pixel_pos[1] - current_pixel_pos[1]  # 列差(东西方向)
    pixel_dy = target_pixel_pos[0] - current_pixel_pos[0]  # 行差(南北方向)

    # 计算当前距离
    pixel_distance = math.sqrt(pixel_dx**2 + pixel_dy**2)
    real_distance_meters = pixel_distance / scale_factor * robot.DISTANCE_PER_GRID_CELL

    logger.info(
        f"精确移动前: 当前位置 {current_pixel_pos}，与目标点 {target_pixel_pos} 之间的欧氏距离: {pixel_distance:.2f} 像素，实际距离: {real_distance_meters:.3f} 米"
    )

    print(f"像素移动向量: dx={pixel_dx}, dy={pixel_dy}")

    # 将像素距离转换为实际移动距离(以DISTANCE_PER_GRID_CELL为单位)
    # 每个scale_factor像素对应1个DISTANCE_PER_GRID_CELL
    actual_dx_steps = pixel_dx / scale_factor  # 东西方向的步数(可以是小数)
    actual_dy_steps = pixel_dy / scale_factor  # 南北方向的步数(可以是小数)

    print(
        f"实际移动步数: dx_steps={actual_dx_steps:.3f}, dy_steps={actual_dy_steps:.3f}"
    )

    # 优先处理较大的移动方向
    movements = []

    # 南北方向移动
    if abs(actual_dy_steps) > 0.05:  # 大于0.05步才移动
        if actual_dy_steps > 0:  # 向南移动
            movements.append((abs(actual_dy_steps), "south"))
        else:  # 向北移动
            movements.append((abs(actual_dy_steps), "north"))

    # 东西方向移动
    if abs(actual_dx_steps) > 0.05:  # 大于0.05步才移动
        if actual_dx_steps > 0:  # 向东移动
            movements.append((abs(actual_dx_steps), "east"))
        else:  # 向西移动
            movements.append((abs(actual_dx_steps), "west"))

    # 按移动距离排序，先执行较大的移动
    movements.sort(reverse=True, key=lambda x: x[0])

    print(f"计划的移动序列: {movements}")

    # 保存原始设置
    original_error_correction = robot.enable_error_correction
    original_speed_factor = robot.speed_factor

    # 设置精确移动模式
    robot.enable_error_correction = False
    robot.speed_factor = 0.3  # 使用较低速度提高精度

    try:
        success = True
        for steps, direction in movements:
            print(f"执行精确移动: {steps:.3f} 步 {direction}")

            # 转向指定方向
            if direction == "north":
                robot.turn_north()
            elif direction == "south":
                robot.turn_south()
            elif direction == "west":
                robot.turn_west()
            elif direction == "east":
                robot.turn_east()

            # 计算实际移动距离（米）
            move_distance = steps * robot.DISTANCE_PER_GRID_CELL
            print(f"移动距离: {move_distance:.3f} 米")

            # 执行移动
            if move_distance > 0.01:  # 大于1厘米才移动
                robot.go_forward(move_distance)

                # 等待稳定
                for _ in range(10):
                    if robot.step(robot.timestep) == -1:
                        success = False
                        break

            if not success:
                break

    finally:
        # 恢复原始设置
        robot.enable_error_correction = original_error_correction
        robot.speed_factor = original_speed_factor

    print(f"精确移动完成，结果: {'成功' if success else '失败'}")
    return success


def calculate_pixel_distance(pos1, pos2):
    """计算两个像素位置之间的距离"""
    dx = pos2[1] - pos1[1]
    dy = pos2[0] - pos1[0]
    return math.sqrt(dx * dx + dy * dy)


# --------------------------------------------------------------抓取-------------------------------------------------
# 创建 Robot 实例
# robot = Robot()

# 创建Supervisor实例，因为你的urdf_arm节点设置了supervisor TRUE
supervisor = Supervisor()

# 获取当前世界的仿真步长
timestep = int(robot.getBasicTimeStep())
vacuum = robot.getDevice("right_gripper_vacuum")


# --- 辅助函数 ---
def convert_to_list(value):
    """将NumPy数组转换为列表，其他类型保持不变"""
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


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
        print(
            f"警告: 右臂关节位置传感器数量({len(right_arm_sensors)})与关节数量({len(right_arm_joint_names)})不匹配"
        )

    # 读取每个关节传感器的值
    for i, sensor in enumerate(right_arm_sensors):
        position = sensor.getValue()
        positions.append(position)
        print(f"关节 {right_arm_joint_names[i]} 的实际位置: {position:.6f} 弧度")

    return positions


def set_arm_joint_positions_with_feedback(
    motors, sensors, targets, tolerance=0.01, max_steps=1000
):
    """带反馈的关节位置控制，确保达到目标位置"""
    if len(targets) != len(motors):
        print(f"错误: 目标位置数量({len(targets)})与电机数量({len(motors)})不匹配")
        return False, []  # 返回元组，避免后续解包错误

    print(f"设置关节到目标位置: {targets}")

    # 执行闭环反馈控制
    success = move_with_feedback(motors, sensors, targets, tolerance, max_steps // 2)

    # 获取最终位置（无论成功与否）
    current = get_actual_joint_positions()

    if success:
        print(f"成功到达目标位置，实际位置: {current}")
    else:
        print(f"未能精确到达目标位置，实际位置: {current}")

    print(f"success={success}, current={current}")
    return success, current


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
            print(
                f"警告: 位置参数数量({len(positions)})与夹爪电机数量({len(gripper_motors_list)})不匹配"
            )
            # 使用默认角度填充或截断位置列表
            position_list = positions[: len(gripper_motors_list)] + [
                default_position
            ] * (len(gripper_motors_list) - len(positions))
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
    new_pos = [current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz]

    # 设置新位置
    translation_field.setSFVec3f(new_pos)
    print(f"机器人位置已更新为: {new_pos}")


# --- 主控制序列 ---
def run_arm_sequence(goal_arm):
    print("\n--- 开始UrdfArm右臂抓取放置任务 ---")

    # 1. 移动右臂到初始姿态并打开右夹爪
    if goal_arm == "left" or goal_arm == "both":
        print("步骤1: 移动zuo臂到初始姿态并打开夹爪")
        # set_arm_joint_positions_with_feedback(
        #     left_arm_motors, left_arm_sensors, initial_left_arm_pose,
        #     tolerance=0.02, max_steps=8
        # )
        set_gripper_position(left_gripper_motors, OPEN_GRIPPER_POS)
        robot.step(MOVE_DURATION_STEPS)
        # time.sleep(1)
    if goal_arm == "right" or goal_arm == "both":
        print("步骤1: 移动右臂到初始姿态并打开夹爪")
        # set_arm_joint_positions_with_feedback(
        #     right_arm_motors, right_arm_sensors, initial_right_arm_pose,
        #     tolerance=0.02, max_steps=8
        # )
        set_gripper_position(right_gripper_motors, OPEN_GRIPPER_POS)
        robot.step(MOVE_DURATION_STEPS)
        # time.sleep(1)

    # # 2. 移动右臂到预抓取姿态
    # print("步骤2.1: 移动右臂到预抓取姿态")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )
    # time.sleep(3)

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
    if goal_arm == "left" or goal_arm == "both":
        print("步骤3: 移动zuo臂到抓取姿态（靠近物体）")
        set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            grasp_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
        # time.sleep(10)
        robot.step(7 * MOVE_DURATION_STEPS)
    if goal_arm == "right" or goal_arm == "both":
        print("步骤3: 移动右臂到抓取姿态（靠近物体）")
        set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            grasp_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
        # time.sleep(10)
        robot.step(7 * MOVE_DURATION_STEPS)

    # 4. 夹紧物体
    print("步骤4: 关闭右夹爪抓取物体")
    vacuum.enablePresence(timestep)
    vacuum.turnOn()
    set_gripper_position(right_gripper_motors, CLOSE_GRIPPER_POS)
    # robot.step(MOVE_DURATION_STEPS)
    # time.sleep(1)

    # # 5. 抬起物体（回到预抓取姿态）
    # print("步骤5: 抬起物体（回到预抓取姿态）")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )

    # 7. 移动右臂到放置姿态
    if goal_arm == "left" or goal_arm == "both":
        print("步骤6: 移动zuo臂到放置姿态（目标位置）")
        # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
        robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
        set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            place_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    if goal_arm == "right" or goal_arm == "both":
        print("步骤6: 移动右臂到放置姿态（目标位置）")
        # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
        robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
        set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            place_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    # time.sleep(1)

    # # 6. 移动右臂到放置前的预备姿态
    # print("步骤7: 移动右臂到放置前预备姿态")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_place_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )
    # # time.sleep(1)

    # 8. 松开物体
    print("步骤8: 打开右夹爪释放物体")
    vacuum.enablePresence(timestep)
    vacuum.turnOff()
    set_gripper_position(right_gripper_motors, OPEN_GRIPPER_POS)
    # robot.step(MOVE_DURATION_STEPS)
    # time.sleep(1)
    set_gripper_position(right_gripper_motors, INIT_GRIPPER_POS)
    print("\n--- 抓取放置任务完成 ---")

    # # 9.
    # print("步骤9: 移动右臂到预抓取姿态")
    # set_arm_joint_positions_with_feedback(
    #     right_arm_motors, right_arm_sensors, pre_grasp_right_arm_pose,
    #     tolerance=0.02, max_steps=80
    # )
    # time.sleep(1)


def run_arm_pick(goal_arm):
    # 初始化变量
    current_left = None
    current_right = None
    success_left = False
    success_right = False
    # print("\n--- 开始UrdfArm右臂抓取放置任务 ---")

    # # 1. 移动右臂到初始姿态并打开右夹爪
    if goal_arm == "left" or goal_arm == "both":
        print("步骤1: 移动zuo臂到初始姿态并打开夹爪")
        # set_arm_joint_positions_with_feedback(
        #     left_arm_motors, left_arm_sensors, initial_left_arm_pose,
        #     tolerance=0.02, max_steps=8
        # )
        set_gripper_position(left_gripper_motors, OPEN_GRIPPER_POS)
        robot.step(MOVE_DURATION_STEPS)
        # time.sleep(1)
    if goal_arm == "right" or goal_arm == "both":
        print("步骤1: 移动右臂到初始姿态并打开夹爪")
        # set_arm_joint_positions_with_feedback(
        #     right_arm_motors, right_arm_sensors, initial_right_arm_pose,
        #     tolerance=0.02, max_steps=8
        # )
        set_gripper_position(right_gripper_motors, OPEN_GRIPPER_POS)
        robot.step(MOVE_DURATION_STEPS)
        # time.sleep(1)

    # 3. 移动右臂到抓取姿态
    if goal_arm == "left" or goal_arm == "both":
        print("步骤3: 移动zuo臂到抓取姿态（靠近物体）")
        success_left, current_left = set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            grasp_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
        # time.sleep(10)
        # robot.step(7*MOVE_DURATION_STEPS)
    if goal_arm == "right" or goal_arm == "both":
        print("步骤3: 移动右臂到抓取姿态（靠近物体）")
        success_right, current_right = set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            grasp_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
        # time.sleep(10)
        # robot.step(5 * MOVE_DURATION_STEPS)

    # 4. 夹紧物体
    print("步骤4: 关闭右夹爪抓取物体")
    vacuum.enablePresence(timestep)
    vacuum.turnOn()
    set_gripper_position(right_gripper_motors, CLOSE_GRIPPER_POS)
    set_gripper_position(left_gripper_motors, CLOSE_GRIPPER_POS)

    # robot.step(MOVE_DURATION_STEPS)
    # time.sleep(1)

    print("步骤5: 抬起物体（回到save姿态）")
    if goal_arm == "left" or goal_arm == "both":
        set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            save_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    if goal_arm == "right" or goal_arm == "both":
        set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            save_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )

    return current_left, current_right


def run_arm_place(goal_arm):
    # 初始化变量
    current_left = None
    current_right = None
    success_left = False
    success_right = False
    # print("\n--- 开始UrdfArm右臂抓取放置任务 ---")

    # 7. 移动右臂到放置姿态
    if goal_arm == "left" or goal_arm == "both":
        print("步骤6: 移动zuo臂到放置姿态（目标位置）")
        # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
        robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
        success_left, current_left = set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            place_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    if goal_arm == "right" or goal_arm == "both":
        print("步骤6: 移动右臂到放置姿态（目标位置）")
        # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
        robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
        success_right, current_right = set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            place_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    # time.sleep(1)

    # 8. 松开物体
    print("步骤8: 打开右夹爪释放物体")
    vacuum.enablePresence(timestep)
    vacuum.turnOff()
    set_gripper_position(right_gripper_motors, OPEN_GRIPPER_POS)
    set_gripper_position(left_gripper_motors, OPEN_GRIPPER_POS)
    # robot.step(MOVE_DURATION_STEPS)
    # time.sleep(1)
    # set_gripper_position(right_gripper_motors, INIT_GRIPPER_POS)
    # set_gripper_position(left_gripper_motors, INIT_GRIPPER_POS)
    print("步骤5: 抬起物体（回到save姿态）")
    if goal_arm == "left" or goal_arm == "both":
        set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            save_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    if goal_arm == "right" or goal_arm == "both":
        set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            save_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
    print("\n--- 抓取放置任务完成 ---")

    return current_left, current_right


def get_pick_result(goal_arm="both"):
    """
    根据指定机械臂判断抓取是否成功，通过检查夹爪关节位置是否处于关闭状态
    TODO:需要修改判断逻辑

    Args:
        goal_arm: 目标机械臂，可选"left"、"right"或"both"，默认"right"
    """
    results = {}

    # 处理单个机械臂的检查逻辑
    def check_arm(arm_name):
        if arm_name == "left":
            gripper_sensors = left_gripper_sensors
            close_positions = CLOSE_GRIPPER_POS
        elif arm_name == "right":
            gripper_sensors = right_gripper_sensors
            close_positions = CLOSE_GRIPPER_POS
        else:
            print(f"错误: 不支持的机械臂类型: {arm_name}")
            return False

        # 检查传感器是否存在
        if not gripper_sensors:
            print(f"错误: 未找到{arm_name}夹爪位置传感器，无法判断抓取状态")
            return False

        # 获取夹爪当前关节位置
        current_positions = [sensor.getValue() for sensor in gripper_sensors]
        print(
            f"{arm_name}夹爪当前关节位置: {[round(pos, 4) for pos in current_positions]}"
        )

        # 定义位置偏差阈值（允许的最大偏差弧度）
        position_threshold = 0.1

        # 检查每个关节是否接近关闭位置
        is_closed = True
        for i, pos in enumerate(current_positions):
            if i < len(close_positions):
                close_pos = close_positions[i]
                if abs(pos - close_pos) > position_threshold:
                    is_closed = False
                    break
            else:
                print(f"警告: 关闭位置定义不足，缺少第{i}个关节的目标值")
                is_closed = False
                break

        # 生成结果信息
        status = "成功" if is_closed else "失败"
        print(
            f"{arm_name}机械臂抓取结果: {status} (夹爪{'已关闭' if is_closed else '未关闭'})"
        )
        return is_closed

    # 根据goal_arm参数执行不同的检查策略
    if goal_arm == "both":
        results["left"] = check_arm("left")
        results["right"] = check_arm("right")
        # 只有当左右机械臂都成功时，总结果才为True
        total_result = all(results.values())
        print(f"双机械臂抓取总结果: {'成功' if total_result else '失败'}")
        return total_result
    else:
        return check_arm(goal_arm)


def get_place_result(goal_arm="both"):
    """
    根据指定机械臂判断放置操作是否成功（夹爪是否处于打开状态）
    TODO:需要修改判断逻辑

    Args:
        goal_arm: 目标机械臂，可选"left"、"right"或"both"，默认"right"
    """
    results = {}

    # 处理单个机械臂的检查逻辑
    def check_arm(arm_name):
        if arm_name == "left":
            gripper_sensors = left_gripper_sensors
        elif arm_name == "right":
            gripper_sensors = right_gripper_sensors
        else:
            print(f"错误: 不支持的机械臂类型: {arm_name}")
            return False

        # 检查传感器是否存在
        if not gripper_sensors:
            print(f"错误: 未找到{arm_name}夹爪位置传感器，无法判断放置状态")
            return False

        # 获取夹爪当前关节位置
        current_positions = [sensor.getValue() for sensor in gripper_sensors]
        print(
            f"{arm_name}夹爪当前关节位置: {[round(pos, 4) for pos in current_positions]}"
        )

        # 定义位置偏差阈值（允许的最大偏差弧度）
        position_threshold = 0.1

        # 检查每个关节是否接近打开位置
        is_open = True
        for i, pos in enumerate(current_positions):
            if i < len(OPEN_GRIPPER_POS):
                open_pos = OPEN_GRIPPER_POS[i]
                if abs(pos - open_pos) > position_threshold:
                    is_open = False
                    break
            else:
                print(f"警告: 打开位置定义不足，缺少第{i}个关节的目标值")
                is_open = False
                break

        # 生成结果信息
        status = "成功" if is_open else "失败"
        print(
            f"{arm_name}机械臂放置结果: {status} (夹爪{'已打开' if is_open else '未打开'})"
        )
        return is_open

    # 根据goal_arm参数执行不同的检查策略
    if goal_arm == "both":
        results["left"] = check_arm("left")
        results["right"] = check_arm("right")
        # 只有当左右机械臂都成功打开时，总结果才为True
        total_result = all(results.values())
        print(f"双机械臂放置总结果: {'成功' if total_result else '失败'}")
        return total_result
    else:
        return check_arm(goal_arm)


def arm_to_go(goal_arm="both"):
    if goal_arm == "left" or goal_arm == "both":
        print("步骤6: 移动zuo臂到放置姿态（目标位置）")
        # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
        robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
        success_left, current_left = set_arm_joint_positions_with_feedback(
            left_arm_motors,
            left_arm_sensors,
            place_left_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
        print("left arm move end")
    if goal_arm == "right" or goal_arm == "both":
        print("步骤6: 移动右臂到放置姿态（目标位置）")
        # move_robot_linear(dx=-0.5, dy=0.0, dz=0.0)
        robot.step(MOVE_DURATION_STEPS)  # 等待一步让仿真更新
        success_right, current_right = set_arm_joint_positions_with_feedback(
            right_arm_motors,
            right_arm_sensors,
            place_right_arm_pose,
            tolerance=0.02,
            max_steps=80,
        )
        print("right arm move end")

    return current_left, current_right


# --- 定义机械臂、夹爪和头部关节名称 ---
# 根据 Webots 控制台输出的实际设备名称进行定义

# 左臂关节 (7个自由度) - 主要控制对象
left_arm_joint_names = [
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "left_intermediate_joint",
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
    "right_intermediate_joint",
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
head_joint_names = ["head", "head_pitch_motor"]

print("Initializing robot devices...")
left_arm_motors, left_arm_sensors = get_motors_and_sensors(left_arm_joint_names)
left_gripper_motors, left_gripper_sensors = get_motors_and_sensors(
    left_gripper_joint_names
)
right_arm_motors, right_arm_sensors = get_motors_and_sensors(right_arm_joint_names)
right_gripper_motors, right_gripper_sensors = get_motors_and_sensors(
    right_gripper_joint_names
)
head_motors, head_sensors = get_motors_and_sensors(head_joint_names)
print("Device initialization complete.")

# --- 硬编码关节目标位置 ---
# 所有角度单位为弧度，需根据实际仿真环境调试

initial_left_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 初始垂直姿态
pre_grasp_left_arm_pose = [
    0,
    0,
    0,
    0,
    0,
    0,
]  # 准备位置 xyz:[0.2374993  ,-0.33825069 , 0.22093117]     RPY:[-3.02456926, -0.00675474,  0.09522905]
pre_grasp_left_arm_pose_2 = [0, 0, 0, 0, 0, 0]
grasp_left_arm_pose = [0, 0, 0, 0, 0, 0]  # xyz[0.2374993  ,-0.33825069 , 0.10093117]
pre_place_left_arm_pose = [
    0,
    0,
    0,
    0,
    0,
    0,
]  # 放置前预备姿态 xyz[0.2374993  ,-0.38825069 , 0.20093117]
place_left_arm_pose = [
    0.0,
    -0.8,
    0.0,
    0.0,
    0.0,
    0,
]  # 放置姿态 xyz[0.24993  ,-0.38825069 , 0.13093117]

initial_right_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0]  # 初始垂直姿态
pre_grasp_right_arm_pose = [
    -1.97400184e00,
    8.07771850e-02,
    0,
    1.83056540e00,
    6.98207363e-02,
    0,
]  # 准备位置 xyz:[0.2374993  ,-0.33825069 , 0.22093117]     RPY:[-3.02456926, -0.00675474,  0.09522905]
pre_grasp_right_arm_pose_2 = [
    -1.97400184e00,
    8.07771850e-02,
    0,
    1.83056540e00,
    6.98207363e-02,
    0,
]
grasp_right_arm_pose = [
    -1.7,
    -4.55066382e-02,
    1.69054925e-01,
    1.47089804e00,
    4.53473656e-02,
    0,
]  # xyz[0.2374993  ,-0.33825069 , 0.10093117]
pre_place_right_arm_pose = [
    -2.3,
    8.07771850e-02,
    0,
    1.83056540e00,
    6.98207363e-02,
    0,
]  # 放置前预备姿态 xyz[0.2374993  ,-0.38825069 , 0.20093117]
place_right_arm_pose = [
    0.0,
    -0.8,
    0.0,
    0.0,
    0.0,
    0,
]  # 放置姿态 xyz[0.24993  ,-0.38825069 , 0.13093117]
save_right_arm_pose = [
    -2.13680205,
    -0.18294627,
    -0.91404125,
    1.5862723,
    -0.36505309,
    -0.83024041,
]
save_left_arm_pose = [
    -2.1527362,
    0.13005413,
    0.85950687,
    1.53729901,
    0.35573519,
    0.89010668,
]
# 右夹爪控制值 - 需根据夹爪模型调试
OPEN_GRIPPER_POS = [-0.7, 0.7, -0.7, -0.7, 0.7, 0.7]  # 夹爪完全打开位置
# CLOSE_GRIPPER_POS = [0.25,-0.25,0.25,0.25,-0.25,-0.25]  # 夹爪夹紧位置
CLOSE_GRIPPER_POS = [0.3, -0.3, 0.3, 0.3, -0.3, -0.3]  # 夹爪夹紧位置
INIT_GRIPPER_POS = [0, 0, 0, 0, 0, 0]

# 运动时间步长
MOVE_DURATION_STEPS = 500  # 每次动作等待的时间步

# 主循环:
current_robot_position = start
path = []
current_step_index = 0
goal_received = False
last_goal_timestamp = None  # 添加时间戳跟踪
rotation_test_done = False  # 用于标记旋转测试是否已完成
touch_sensor = robot.getDevice("right_suction_touch_sensor")
touch_sensor.enable(timestep)
counter_1 = 0
while robot.step(timestep) != -1:
    try:
        # touchValues = touch_sensor.getValue()  # 返回标量值，适用于默认类型
        # if touchValues > 0:
        #     print("Robot is touching an object.")
        # else:
        #     print("no touch")
        time.sleep(0.015)
        counter_1 = counter_1 + 1
        if counter_1 == 100:
            counter_1 = 0
            continue
        response = requests.get(WEB_SERVER_URL)
        if response.status_code == 200:
            command_data = response.json()
            source = command_data.get('source')
            if source == "robot_goals":
                logger.info(command_data.get("goal", []))
                if not command_data:  # 如果返回空数据，说明是停止命令
                    if goal_received:
                        logger.info("收到停止命令，清空当前路径和目标")
                        goal_received = False
                        path = []
                        current_step_index = 0
                        # 发送状态更新
                        stop_info = {
                            "status": "已停止",
                            "message": "机器人已停止移动",
                            "is_completed": False,
                            "is_stopped": True,
                        }
                        send_robot_status(current_robot_position, stop_info)
                    continue
                elif len(command_data.get("goal", [])) == 0:
                    logger.info("AAAAAAAAAAAAAAAAAAAAAAA")
                    new_goal_yaw = command_data.get("goal_angle", 0.0)
                    new_goal_timestamp = command_data.get("goal_timestamp", time.time())
                    if (
                        new_goal_yaw != goal_yaw
                        or last_goal_timestamp != new_goal_timestamp
                    ):
                        robot.stop_wheels()
                        goal_received = False
                        path = []
                        current_step_index = 0
                        goal_yaw = new_goal_yaw
                        last_goal_timestamp = new_goal_timestamp
                        logger.info(f"收到目标偏航角: {goal_yaw}度")
                        # 只执行原地旋转
                        current_heading = robot.get_current_angle_from_imu()
                        target_heading = math.radians(goal_yaw)
                        angle_diff = (
                            (target_heading - current_heading + math.pi) % (2 * math.pi)
                        ) - math.pi
                        robot.rotate_angle(angle_diff)
                        # 发送旋转完成状态
                        rotation_info = {
                            "status": "旋转完成",
                            "message": f"已完成原地旋转至目标偏航角 {goal_yaw}°",
                            "is_completed": True,
                            "is_stopped": True
                        }
                        send_robot_status(current_robot_position, rotation_info)
                    continue
                if "goal" in command_data and command_data["goal"]:
                    try:
                        new_goal = command_data.get("goal", [])
                        new_goal_yaw = command_data.get(
                            "goal_angle", 0.0
                        )  # 获取目标偏航角，默认为0度
                        new_goal_timestamp = command_data.get(
                            "goal_timestamp", time.time()
                        )  # 获取目标时间戳

                        if len(new_goal) >= 2:  # 确保目标坐标至少有两个元素
                            print(new_goal)
                            new_goal = map_transfor.webots_to_map(new_goal)
                            print(new_goal)
                            if path:
                                print(f"当前路径: {path}")

                            is_new_goal = (
                                new_goal != goal
                                or new_goal_yaw != goal_yaw
                                or new_goal_timestamp != last_goal_timestamp
                            )

                            if is_new_goal:
                                robot.stop_wheels()  # 立即停止轮子转动
                                print(f"检测到新目标，停止当前移动并重置状态")
                                print(
                                    f"目标变化原因: 位置{'不同' if new_goal != goal else '相同'}, "
                                    f"角度{'不同' if new_goal_yaw != goal_yaw else '相同'}, "
                                    f"时间戳{'不同' if new_goal_timestamp != last_goal_timestamp else '相同'}"
                                )

                                goal_received = False
                                path = []
                                current_step_index = 0

                                goal = new_goal
                                goal_yaw = new_goal_yaw
                                last_goal_timestamp = new_goal_timestamp  # 更新时间戳
                                print(f"收到新目标点: {goal}, 目标偏航角: {goal_yaw}度")
                                print("开始路径规划...")
                                time_start = time.time()
                                # 传递机器人ID给BFS_planning函数
                                current_object_id = command_data.get("object_id", 1)
                                path = BFS_planning(
                                    refined_world_map,
                                    current_robot_position,
                                    goal,
                                    current_object_id,
                                )
                                print(
                                    "BFS路径规划完成，耗时: {:.2f} 秒".format(
                                        time.time() - time_start
                                    )
                                )

                                # 检查路径规划结果
                                if path:
                                    print(f"简化路径: {path}")
                                    current_step_index = 0
                                    goal_received = True
                                    # 更新状态信息，添加路径总段数和当前段索引
                                    status_info = {
                                        "status": "success",
                                        "message": f"已找到到达目标 {goal} 的路径",
                                        "path_length": len(path),
                                        "total_segments": len(path),
                                        "current_segment": current_step_index,
                                        "object_id": command_data.get("object_id", 1),
                                        "is_reachable": True,
                                        "path_planning_complete": True,  # 明确标记路径规划已完成且成功
                                    }
                                    send_robot_status(
                                        current_robot_position, status_info
                                    )
                                else:
                                    goal_received = False
                                    goal = None
                                    path = []
                        else:
                            print(f"警告: 收到无效的目标格式: {new_goal}")
                    except (TypeError, IndexError) as e:
                        print(
                            f"处理目标坐标时出错: {e}, 收到的数据: {command_data.get('goal')}"
                        )
            elif source == "robot_capture":
                # print("command_data",command_data)
                # print("command_data[object_pos]",command_data.get('right_cur_pos'))
                # print(command_data.get('right_goal_pos'))
                goal_arm = command_data.get('goal_arm')
                status_info = {"status": "success", "task": "capture"}
                send_robot_status(current_robot_position, status_info)
                # 初始化双臂规划器
                planner = ArmIk(
                    model_file="../../../Webots_PR2_Path_Planning/protos/urdf_arm_mix/urdf/urdf_arm_mix.urdf"
                )
                # 创建机器人运动学控制器实例，bu启用可视化
                arm_ik = ArmIk(visualize=False)

                # 获取初始关节角度
                q0 = arm_ik.get_init_q()

                # 1.准备位置
                # 定义目标位置
                # l_hand_pose = np.array([-0.01749985,0.29927,-0.21073])  # [x, y, z] 单位m
                # r_hand_pose = np.array([0.2374993  ,-0.33825069 , 0.22093117])

                # # 求解双臂IK
                # sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, np.array([3.14, 0, 0]),np.array([-3.02456926, -0.00675474,  0.09522905]))
                # # print("sol_q1111111111111111:",sol_q)
                # pre_grasp_right_arm_pose = sol_q[19:24]
                # # print("pre_grasp_right_arm_pose:",pre_grasp_right_arm_pose)
                # # pre_grasp_right_arm_pose_2 = pre_grasp_right_arm_pose
                # # pre_grasp_right_arm_pose_2[5] = 1.5
                # # print("pre_grasp_right_arm_pose_2:",pre_grasp_right_arm_pose_2)

                # 2.抓取位置
                # print("command_data.get('goal_pos'):",command_data.get('goal_pos'))
                l_hand_pose = np.array(command_data.get('left_cur_pos'))
                r_hand_pose = np.array(command_data.get('right_cur_pos'))
                l_hand_rpy = np.array(command_data.get('left_angle'))
                r_hand_rpy = np.array(command_data.get('right_angle'))

                if goal_arm == "left":
                    r_hand_pose = [0.0, -0.0, 0.0]
                    r_hand_rpy = [3.14, 0, 0]
                elif goal_arm == "right":
                    l_hand_pose = [0.0, 0.0, 0.0]
                    l_hand_rpy = [-3.14, -0.00675474, 0.09522905]

                print("l_hand_pose:", l_hand_pose)
                print("r_hand_pose:", r_hand_pose)
                sol_q = arm_ik.computeIK(
                    q0, l_hand_pose, r_hand_pose, l_hand_rpy, r_hand_rpy
                )
                print("sol_q22222222222222:", sol_q)
                if goal_arm == "left" or goal_arm == "both":
                    grasp_left_arm_pose = sol_q[7:13]
                if goal_arm == "right" or goal_arm == "both":
                    grasp_right_arm_pose = sol_q[19:25]
                # grasp_right_arm_pose = [-1.7, -4.55066382e-02 , 1.69054925e-01 , 1.47089804e+00  ,4.53473656e-02]
                print("grasp_left_arm_pose:", grasp_left_arm_pose)
                print("grasp_right_arm_pose:", grasp_right_arm_pose)

                # 3.移动并放置
                l_hand_pose = np.array(command_data.get('left_goal_pos'))
                r_hand_pose = np.array(command_data.get('right_goal_pos'))
                l_hand_rpy = np.array(command_data.get('left_angle'))
                r_hand_rpy = np.array(command_data.get('right_angle'))
                print("l_hand_pose:", l_hand_pose)
                print("r_hand_pose:", r_hand_pose)
                sol_q = arm_ik.computeIK(
                    q0, l_hand_pose, r_hand_pose, l_hand_rpy, r_hand_rpy
                )
                print("sol_q33333333333333:", sol_q)
                if goal_arm == "left" or goal_arm == "both":
                    place_left_arm_pose = sol_q[7:12]
                if goal_arm == "right" or goal_arm == "both":
                    place_right_arm_pose = sol_q[18:23]
                print("place_left_arm_pose:", place_left_arm_pose)
                print("place_right_arm_pose:", place_right_arm_pose)
                # 这里添加你的控制逻辑
                run_arm_sequence(goal_arm)
            elif source == "robot_pick":
                goal_arm = command_data.get('goal_arm')
                goal_arm
                # 初始化双臂规划器
                planner = ArmIk(
                    model_file="../../../Webots_PR2_Path_Planning/protos/urdf_arm_mix/urdf/urdf_arm_mix.urdf"
                )
                # 创建机器人运动学控制器实例，bu启用可视化
                arm_ik = ArmIk(visualize=False)

                # 获取初始关节角度
                q0 = arm_ik.get_init_q()

                # 1.准备位置
                # 定义目标位置
                # l_hand_pose = np.array([-0.01749985,0.29927,-0.21073])  # [x, y, z] 单位m
                # r_hand_pose = np.array([0.2374993  ,-0.33825069 , 0.22093117])

                # # 求解双臂IK
                # sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, np.array([3.14, 0, 0]),np.array([-3.02456926, -0.00675474,  0.09522905]))
                # # print("sol_q1111111111111111:",sol_q)
                # pre_grasp_right_arm_pose = sol_q[19:24]
                # # print("pre_grasp_right_arm_pose:",pre_grasp_right_arm_pose)
                # # pre_grasp_right_arm_pose_2 = pre_grasp_right_arm_pose
                # # pre_grasp_right_arm_pose_2[5] = 1.5
                # # print("pre_grasp_right_arm_pose_2:",pre_grasp_right_arm_pose_2)

                # 2.抓取位置
                # print("command_data.get('goal_pos'):",command_data.get('goal_pos'))
                l_hand_pose = np.array(command_data.get('left_cur_pos'))
                r_hand_pose = np.array(command_data.get('right_cur_pos'))
                l_hand_rpy = np.array(command_data.get('left_angle'))
                r_hand_rpy = np.array(command_data.get('right_angle'))

                if goal_arm == "left":
                    r_hand_pose = [0.0, -0.0, 0.0]
                    r_hand_rpy = [3.14, 0, 0]
                elif goal_arm == "right":
                    l_hand_pose = [0.0, 0.0, 0.0]
                    l_hand_rpy = [-3.14, -0.00675474, 0.09522905]

                print("l_hand_pose:", l_hand_pose)
                print("r_hand_pose:", r_hand_pose)
                sol_q = arm_ik.computeIK(
                    q0, l_hand_pose, r_hand_pose, l_hand_rpy, r_hand_rpy
                )
                print("sol_q22222222222222:", sol_q)
                if sol_q is None:
                    status_info = {
                        "status": "fail",
                        "task": "pick",
                    }
                else:
                    if goal_arm == "left" or goal_arm == "both":
                        grasp_left_arm_pose = sol_q[7:13]
                    if goal_arm == "right" or goal_arm == "both":
                        grasp_right_arm_pose = sol_q[19:25]
                    # grasp_right_arm_pose = [-1.7, -4.55066382e-02 , 1.69054925e-01 , 1.47089804e+00  ,4.53473656e-02]
                    print("grasp_left_arm_pose:", grasp_left_arm_pose)
                    print("grasp_right_arm_pose:", grasp_right_arm_pose)

                    # 3.移动并放置
                    # l_hand_pose = np.array(command_data.get('left_goal_pos'))
                    # r_hand_pose = np.array(command_data.get('right_goal_pos'))
                    # l_hand_rpy = np.array(command_data.get('left_angle'))
                    # r_hand_rpy = np.array(command_data.get('right_angle'))
                    # print("l_hand_pose:",l_hand_pose)
                    # print("r_hand_pose:",r_hand_pose)
                    # sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, l_hand_rpy,r_hand_rpy)
                    # print("sol_q33333333333333:",sol_q)
                    # if goal_arm == "left" or goal_arm == "both":
                    #     place_left_arm_pose = sol_q[7:12]
                    # if goal_arm == "right" or goal_arm == "both":
                    #     place_right_arm_pose = sol_q[18:23]
                    # print("place_left_arm_pose:",place_left_arm_pose)
                    # print("place_right_arm_pose:",place_right_arm_pose)
                    # 这里添加你的控制逻辑
                    current_left_joint, current_right_joint = run_arm_pick(goal_arm)
                    # print("current_left,current_right", current_left, current_right)
                    current_left_pos = []
                    current_right_pos = []
                    if goal_arm == "left":
                        # 注意：此处应传入 current_left_joint
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=current_left_joint, goal_arm="left"
                        )
                        current_left_pos = result["left_position"]

                    elif goal_arm == "right":
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=current_right_joint,  # 修正为 current_right_joint
                            goal_arm="right",
                        )
                        current_right_pos = result["right_position"]

                    elif goal_arm == "both":
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=np.concatenate(
                                [current_left_joint, current_right_joint]
                            ),
                            goal_arm="both",
                        )
                        current_left_pos = result["left_position"]
                        current_right_pos = result["right_position"]

                    res = get_pick_result(goal_arm)
                    current_left_pos = convert_to_list(current_left_pos)
                    current_right_pos = convert_to_list(current_right_pos)
                    print("current_left_pos", current_left_pos)
                    print("current_right_pos", current_right_pos)
                    status_info = {
                        "status": "success",
                        "task": "pick",
                        "res": res,
                        "current_left_pos": current_left_pos,
                        "current_right_pos": current_right_pos,
                    }
                send_robot_status(current_robot_position, status_info)
            elif source == "robot_place":
                goal_arm = command_data.get('goal_arm')
                # 初始化双臂规划器
                planner = ArmIk(
                    model_file="../../../Webots_PR2_Path_Planning/protos/urdf_arm_mix/urdf/urdf_arm_mix.urdf"
                )
                # 创建机器人运动学控制器实例，bu启用可视化
                arm_ik = ArmIk(visualize=False)

                # 获取初始关节角度
                q0 = arm_ik.get_init_q()

                # 1.准备位置
                # 定义目标位置
                # l_hand_pose = np.array([-0.01749985,0.29927,-0.21073])  # [x, y, z] 单位m
                # r_hand_pose = np.array([0.2374993  ,-0.33825069 , 0.22093117])

                # # 求解双臂IK
                # sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, np.array([3.14, 0, 0]),np.array([-3.02456926, -0.00675474,  0.09522905]))
                # # print("sol_q1111111111111111:",sol_q)
                # pre_grasp_right_arm_pose = sol_q[19:24]
                # # print("pre_grasp_right_arm_pose:",pre_grasp_right_arm_pose)
                # # pre_grasp_right_arm_pose_2 = pre_grasp_right_arm_pose
                # # pre_grasp_right_arm_pose_2[5] = 1.5
                # # print("pre_grasp_right_arm_pose_2:",pre_grasp_right_arm_pose_2)

                # 2.抓取位置
                # print("command_data.get('goal_pos'):",command_data.get('goal_pos'))
                # l_hand_pose = np.array(command_data.get('left_cur_pos'))
                # r_hand_pose = np.array(command_data.get('right_cur_pos'))
                # l_hand_rpy = np.array(command_data.get('left_angle'))
                # r_hand_rpy = np.array(command_data.get('right_angle'))

                # print("l_hand_pose:",l_hand_pose)
                # print("r_hand_pose:",r_hand_pose)
                # sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, l_hand_rpy,r_hand_rpy)
                # print("sol_q22222222222222:",sol_q)
                # if goal_arm == "left" or goal_arm == "both":
                #     grasp_left_arm_pose = sol_q[7:12]
                # if goal_arm == "right" or goal_arm == "both":
                #     grasp_right_arm_pose = sol_q[18:23]
                # # grasp_right_arm_pose = [-1.7, -4.55066382e-02 , 1.69054925e-01 , 1.47089804e+00  ,4.53473656e-02]
                # print("grasp_left_arm_pose:",grasp_left_arm_pose)
                # print("grasp_right_arm_pose:",grasp_right_arm_pose)

                # 3.移动并放置
                l_hand_pose = np.array(command_data.get('left_goal_pos'))
                r_hand_pose = np.array(command_data.get('right_goal_pos'))
                l_hand_rpy = np.array(command_data.get('left_angle'))
                r_hand_rpy = np.array(command_data.get('right_angle'))

                if goal_arm == "left":
                    r_hand_pose = [0.0, 0.0, 0.0]
                    r_hand_rpy = [3.14, 0, 0]
                elif goal_arm == "right":
                    l_hand_pose = [0.0, 0.0, 0.0]
                    l_hand_rpy = [-3.14, -0.00675474, 0.09522905]

                print("l_hand_pose:", l_hand_pose)
                print("r_hand_pose:", r_hand_pose)
                sol_q = arm_ik.computeIK(
                    q0, l_hand_pose, r_hand_pose, l_hand_rpy, r_hand_rpy
                )
                print("sol_q33333333333333:", sol_q)
                if sol_q is None:
                    status_info = {
                        "status": "fail",
                        "task": "place",
                    }
                else:
                    if goal_arm == "left" or goal_arm == "both":
                        place_left_arm_pose = sol_q[7:13]
                    if goal_arm == "right" or goal_arm == "both":
                        place_right_arm_pose = sol_q[19:25]
                    print("place_left_arm_pose:", place_left_arm_pose)
                    print("place_right_arm_pose:", place_right_arm_pose)
                    # 这里添加你的控制逻辑
                    current_left_joint, current_right_joint = run_arm_place(goal_arm)
                    current_left_pos = []
                    current_right_pos = []
                    if goal_arm == "left":
                        # 注意：此处应传入 current_left_joint
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=current_left_joint, goal_arm="left"
                        )
                        current_left_pos = result["left_position"]

                    elif goal_arm == "right":
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=current_right_joint,  # 修正为 current_right_joint
                            goal_arm="right",
                        )
                        current_right_pos = result["right_position"]

                    elif goal_arm == "both":
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=np.concatenate(
                                [current_left_joint, current_right_joint]
                            ),
                            goal_arm="both",
                        )
                        current_left_pos = result["left_position"]
                        current_right_pos = result["right_position"]

                    current_left_pos = convert_to_list(current_left_pos)
                    current_right_pos = convert_to_list(current_right_pos)
                    print("current_left_pos", current_left_pos)
                    print("current_right_pos", current_right_pos)
                    res = get_place_result(goal_arm)
                    status_info = {
                        "status": "success",
                        "task": "place",
                        "res": res,
                        "current_left_pos": current_left_pos,
                        "current_right_pos": current_right_pos,
                    }
                send_robot_status(current_robot_position, status_info)
            elif source == "arm_go_pos":
                goal_arm = command_data.get('goal_arm')

                # 初始化双臂规划器
                planner = ArmIk(
                    model_file="../../../Webots_PR2_Path_Planning/protos/urdf_arm_mix/urdf/urdf_arm_mix.urdf"
                )
                # 创建机器人运动学控制器实例，bu启用可视化
                arm_ik = ArmIk(visualize=False)

                # 获取初始关节角度
                q0 = arm_ik.get_init_q()
                # 3.移动并放置
                l_hand_pose = np.array(command_data.get('left_goal_pos'))
                r_hand_pose = np.array(command_data.get('right_goal_pos'))
                l_hand_rpy = np.array(command_data.get('left_angle'))
                r_hand_rpy = np.array(command_data.get('right_angle'))

                if goal_arm == "left":
                    r_hand_pose = [0.0, 0.0, 0.0]
                    r_hand_rpy = [3.14, 0, 0]
                elif goal_arm == "right":
                    l_hand_pose = [0.0, 0.0, 0.0]
                    l_hand_rpy = [-3.14, -0.00675474, 0.09522905]

                print("l_hand_pose:", l_hand_pose)
                print("r_hand_pose:", r_hand_pose)
                sol_q = arm_ik.computeIK(
                    q0, l_hand_pose, r_hand_pose, l_hand_rpy, r_hand_rpy
                )
                print("sol_q33333333333333:", sol_q)
                if sol_q is None:
                    status_info = {
                        "status": "fail",
                        "task": "arm_to_go",
                    }
                else:
                    if goal_arm == "left" or goal_arm == "both":
                        place_left_arm_pose = sol_q[7:13]
                    if goal_arm == "right" or goal_arm == "both":
                        place_right_arm_pose = sol_q[19:25]
                    print("place_left_arm_pose:", place_left_arm_pose)
                    print("place_right_arm_pose:", place_right_arm_pose)
                    # 这里添加你的控制逻辑
                    current_left_joint, current_right_joint = arm_to_go(goal_arm)
                    current_left_pos = []
                    current_right_pos = []
                    if goal_arm == "left":
                        # 注意：此处应传入 current_left_joint
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=current_left_joint, goal_arm="left"
                        )
                        current_left_pos = result["left_position"]

                    elif goal_arm == "right":
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=current_right_joint,  # 修正为 current_right_joint
                            goal_arm="right",
                        )
                        current_right_pos = result["right_position"]

                    elif goal_arm == "both":
                        result = arm_ik.calculate_eef_pose(
                            joint_angles=np.concatenate(
                                [current_left_joint, current_right_joint]
                            ),
                            goal_arm="both",
                        )
                        current_left_pos = result["left_position"]
                        current_right_pos = result["right_position"]

                    current_left_pos = convert_to_list(current_left_pos)
                    current_right_pos = convert_to_list(current_right_pos)
                    print("current_left_pos", current_left_pos)
                    print("current_right_pos", current_right_pos)
                    # res = True
                    status_info = {
                        "status": "success",
                        "task": "arm_to_go",
                        "res": True,
                        "current_left_pos": current_left_pos,
                        "current_right_pos": current_right_pos,
                    }
                send_robot_status(current_robot_position, status_info)
    except requests.exceptions.ConnectionError:
        pass
    except json.JSONDecodeError:
        print("Invalid JSON response from web server.")
    except Exception as e:
        print(f"Error fetching command: {e}")

    if goal_received and path:
        # 计算当前位置与目标的距离
        distance_to_goal = math.sqrt(
            (current_robot_position[0] - goal[0]) ** 2
            + (current_robot_position[1] - goal[1]) ** 2
        )

        # 当距离小于3像素时视为到达目标位置
        if distance_to_goal <= 3:
            real_distance_meters = (
                distance_to_goal / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
            )
            # 使用logger记录到达目标时的欧氏距离
            logger.info(
                f"到达目标位置! 当前位置 {current_robot_position}，与目标点 {goal} 之间的欧氏距离: {distance_to_goal:.2f} 像素，实际距离: {real_distance_meters:.3f} 米"
            )

            # 获取当前机器人的航向角（弧度制）
            current_heading = robot.get_current_angle_from_imu()
            # 将目标偏航角从角度转换为弧度
            target_heading = math.radians(goal_yaw)

            # 计算需要旋转的角度（弧度）
            angle_diff = target_heading - current_heading
            # 将角度差规范化到[-pi, pi]范围内
            angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi

            print(f"正在调整到目标偏航角 {goal_yaw}度...")
            robot.rotate_angle(angle_diff)  # 旋转到目标偏航角

            # 更新状态，表示已达到目标
            completion_info = {
                "status": "完成",
                "message": f"已到达目标位置! 距离: {distance_to_goal:.2f}, 已调整到目标偏航角: {goal_yaw}度",
                "object_id": (
                    command_data.get("object_id", 1)
                    if 'command_data' in locals()
                    else 1
                ),
                "total_segments": len(path),
                "current_segment": len(path),  # 当前段设为总段数，表示已完成
                "progress": 100.0,  # 进度100%
                "is_reachable": True,
                "is_completed": True,
            }
            send_robot_status(current_robot_position, completion_info)
            goal_received = False
            path = []
            current_step_index = 0
            # robot.retract_arms()  # 可选：到达目标后收回机械臂
        else:
            # 如果路径已完成但尚未到达目标点，并且此时没有正在执行的微调，则进行微调
            if current_step_index >= len(path):
                print(
                    f"路径已完成但尚未到达目标点，距离目标还有: {distance_to_goal:.2f} 像素"
                )

                # 获取当前机器人的航向角度
                current_heading = robot.get_current_angle_from_imu()

                # 计算到目标的方向向量
                dx = goal[0] - current_robot_position[0]
                dy = goal[1] - current_robot_position[1]

                # 计算在当前航向上的投影距离（仅在当前朝向上前后移动，不转身）
                heading_vector_x = math.cos(current_heading)
                heading_vector_y = math.sin(current_heading)
                projected_distance = dx * heading_vector_x + dy * heading_vector_y
                projected_distance_meters = (
                    projected_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                )

                # 改用像素级精确移动方法
                print(f"路径已完成但尚未到达目标点，使用像素级精确移动")
                print(f"当前位置: {current_robot_position}, 目标: {goal}")
                print(f"直线距离: {distance_to_goal:.2f} 像素")

                # 尝试使用精确移动到目标
                success = move_to_precise_pixel_position(
                    current_robot_position, goal, SCALE_FACTOR
                )

                if success:
                    # 更新位置并验证
                    gps_position = robot.get_current_position()
                    current_robot_position = map_transfor.webots_to_map(gps_position)
                    final_distance = calculate_pixel_distance(
                        current_robot_position, goal
                    )
                    real_distance_meters = (
                        final_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                    )

                    # 使用logger记录精确移动后的欧氏距离
                    logger.info(
                        f"精确移动完成，当前位置 {current_robot_position}，与目标点 {goal} 之间的欧氏距离: {final_distance:.2f} 像素，实际距离: {real_distance_meters:.3f} 米"
                    )

                    # 如果仍未到达且距离不太大，尝试使用utilities中的精确定位方法
                    if final_distance > 3 and final_distance <= 10:
                        print("尝试使用GPS基础的精确定位方法...")
                        try:
                            precise_success = robot.fine_tune_position_to_pixel(
                                goal, SCALE_FACTOR, tolerance_pixels=3
                            )
                            if precise_success:
                                print("GPS精确定位成功!")
                                gps_position = robot.get_current_position()
                                current_robot_position = map_transfor.webots_to_map(
                                    gps_position
                                )
                                final_distance = calculate_pixel_distance(
                                    current_robot_position, goal
                                )
                        except Exception as e:
                            print(f"GPS精确定位失败: {e}")

                    # 如果距离在可接受范围内，认为已到达
                    if final_distance <= 5:  # 放宽到5像素容差
                        print(f"精确移动成功! 最终距离: {final_distance:.2f} 像素")

                        # 获取当前机器人的航向角（弧度制）
                        current_heading = robot.get_current_angle_from_imu()
                        # 将目标偏航角从角度转换为弧度
                        target_heading = math.radians(goal_yaw)

                        # 计算需要旋转的角度（弧度）
                        angle_diff = target_heading - current_heading
                        # 将角度差规范化到[-pi, pi]范围内
                        angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi

                        print(f"正在调整到目标偏航角 {goal_yaw}度...")
                        robot.rotate_angle(angle_diff)  # 旋转到目标偏航角

                        # 更新状态，表示已达到目标
                        completion_info = {
                            "status": "完成",
                            "message": f"已到达目标位置! 距离: {final_distance:.2f}, 已调整到目标偏航角: {goal_yaw}度",
                            "object_id": (
                                command_data.get("object_id", 1)
                                if 'command_data' in locals()
                                else 1
                            ),
                            "total_segments": len(path),
                            "current_segment": len(
                                path
                            ),  # 当前段设为总段数，表示已完成
                            "progress": 100.0,  # 进度100%
                            "is_reachable": True,
                            "is_completed": True,
                        }
                        send_robot_status(current_robot_position, completion_info)
                        goal_received = False
                        path = []
                        current_step_index = 0
                    else:
                        print(
                            f"精确移动后距离仍较大: {final_distance:.2f} 像素，重新规划路径"
                        )
                        # 重新规划路径
                        print(
                            f"从当前位置 {current_robot_position} 重新规划到目标 {goal}"
                        )
                        new_path = BFS_planning(
                            refined_world_map,
                            current_robot_position,
                            goal,
                            (
                                command_data.get("object_id", 1)
                                if 'command_data' in locals()
                                else 1
                            ),
                        )
                        if new_path:
                            path = new_path
                            current_step_index = 0
                            print(f"重新规划成功，新路径长度: {len(path)}")
                        else:
                            print("重新规划失败，无法到达目标")
                            goal_received = False
                            path = []
                            current_step_index = 0
                else:
                    print("精确移动失败，重新规划路径")

                # 如果微调后仍未到达目标，且距离仍大于3像素，则重新规划路径
                if goal_received and distance_to_goal > 3:
                    print(
                        f"微调后仍未到达目标，距离: {distance_to_goal:.2f}，等待新指令或重新规划路径"
                    )
                    send_robot_status(
                        current_robot_position,
                        f"Still not at goal after adjustment. Distance: {distance_to_goal:.2f}",
                    )

            # 使用化简后的路径进行移动
            if current_step_index < len(path):
                start_pos, steps, direction = path[current_step_index]

                # 获取当前机器人的航向角度
                current_heading = robot.get_current_angle_from_imu()

                # 计算到目标的方向向量
                dx = goal[0] - current_robot_position[0]
                dy = goal[1] - current_robot_position[1]

                # 计算在当前航向上的投影距离（仅在当前朝向上前后移动，不转身）
                heading_vector_x = math.cos(current_heading)
                heading_vector_y = math.sin(current_heading)
                projected_distance = dx * heading_vector_x + dy * heading_vector_y

            # 使用化简后的路径进行移动
            if current_step_index < len(path):
                start_pos, steps, direction = path[current_step_index]

                print(
                    f"Executing step {current_step_index}: {steps} steps {direction} from {start_pos}"
                )
                move_from_to_multi_steps(current_robot_position, steps, direction)

                # 使用robot的GPS获取当前位置
                try:
                    gps_position = robot.get_current_position()
                    current_robot_position = map_transfor.webots_to_map(gps_position)

                    # 打印更详细的位置信信息
                    print(
                        f"当前位置 - GPS: ({gps_position[0]:.3f}, {gps_position[1]:.3f}), 地图: {current_robot_position}"
                    )

                    # 计算与目标的距离
                    if goal:
                        distance_to_goal = math.sqrt(
                            (current_robot_position[0] - goal[0]) ** 2
                            + (current_robot_position[1] - goal[1]) ** 2
                        )
                        real_distance_meters = (
                            distance_to_goal
                            / SCALE_FACTOR
                            * robot.DISTANCE_PER_GRID_CELL
                        )
                        # 使用logger记录当前位置与目标点之间的欧氏距离
                        logger.info(
                            f"路径点 {current_step_index}/{len(path)} 完成，当前位置 {current_robot_position}，与目标点 {goal} 之间的欧氏距离: {distance_to_goal:.2f} 像素，实际距离: {real_distance_meters:.3f} 米"
                        )

                        # 向服务器发送位置和距离信息
                        status_info = {
                            "status": "移动中",
                            "message": f"已完成路径点 {current_step_index}/{len(path)}, 距离目标: {distance_to_goal:.2f} 像素",
                            "object_id": (
                                command_data.get("object_id", 1)
                                if 'command_data' in locals()
                                else 1
                            ),
                            "total_segments": len(path),
                            "current_segment": current_step_index,
                            "progress": (
                                (current_step_index / len(path)) * 100
                                if len(path) > 0
                                else 0
                            ),
                            "distance_to_goal": distance_to_goal,
                            "distance_to_goal_meters": distance_to_goal
                            / SCALE_FACTOR
                            * robot.DISTANCE_PER_GRID_CELL,
                        }
                        send_robot_status(current_robot_position, status_info)
                except Exception as e:
                    print(f"获取位置时出错: {e}")
                    # 如果出错，使用最后一个已知位置
                    print(f"使用最后一个已知位置: {current_robot_position}")

                current_step_index += 1
                # 更新状态，包含总段数和当前段索引，用于进度展示
                progress_info = {
                    "status": "移动中",
                    "message": f"正在沿路径移动，段 {current_step_index}/{len(path)}",
                    "object_id": (
                        command_data.get("object_id", 1)
                        if 'command_data' in locals()
                        else 1
                    ),
                    "total_segments": len(path),
                    "current_segment": current_step_index,
                    "progress": round(
                        current_step_index / len(path) * 100, 1
                    ),  # 百分比进度
                    "is_reachable": True,
                }
                send_robot_status(current_robot_position, progress_info)
            else:
                print("Simplified path completed, checking goal...")
                # 计算当前位置与目标的距离
                distance_to_goal = math.sqrt(
                    (current_robot_position[0] - goal[0]) ** 2
                    + (current_robot_position[1] - goal[1]) ** 2
                )
                logger.info(f"路径已完成，当前距离目标: {distance_to_goal:.2f} 像素")

                # 当距离小于3像素时视为到达目标
                if distance_to_goal <= 3:
                    print(
                        f"Goal reached after simplified path traversal. 距离目标: {distance_to_goal:.2f} (小于等于3)"
                    )
                    send_robot_status(
                        current_robot_position,
                        f"Goal reached! 距离: {distance_to_goal:.2f}",
                    )
                    goal_received = False
                    path = []
                    current_step_index = 0
                else:
                    print(
                        f"Path completed but goal not reached. 距离目标: {distance_to_goal:.2f} (大于3)，进行微调..."
                    )
                    send_robot_status(
                        current_robot_position,
                        f"Distance: {distance_to_goal:.2f}，performing fine adjustment",
                    )

                    # 获取当前机器人的航向角度
                    current_heading = robot.get_current_angle_from_imu()

                    # 计算到目标的方向向量
                    dx_actual = goal[0] - current_robot_position[0]
                    dy_actual = goal[1] - current_robot_position[1]

                    # 计算到目标的实际距离
                    actual_distance = math.sqrt(dx_actual**2 + dy_actual**2)
                    actual_distance_meters = (
                        actual_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                    )

                    # 计算在当前航向上的投影距离（仅在当前朝向上前后移动，不转身）
                    heading_vector_x = math.cos(current_heading)
                    heading_vector_y = math.sin(current_heading)
                    projected_distance = (
                        dx_actual * heading_vector_x + dy_actual * heading_vector_y
                    )
                    projected_distance_meters = (
                        projected_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                    )

                    print(f"当前位置: {current_robot_position}, 目标: {goal}")
                    print(
                        f"直线距离: {actual_distance:.2f} 像素 ({actual_distance_meters:.3f}米)"
                    )
                    print(
                        f"当前航向投影距离: {projected_distance:.2f} 像素 ({projected_distance_meters:.3f}米)"
                    )

                    # 保存原始设置
                    original_error_correction = robot.enable_error_correction
                    original_speed_factor = robot.speed_factor

                    # 关闭误差纠偏，降低速度进行精确微调
                    robot.enable_error_correction = False
                    robot.speed_factor = 0.3  # 低速微调

                    # 仅在当前朝向上进行前后移动微调，不转身
                    if abs(projected_distance) > 1.0:  # 投影距离大于1像素才进行微调
                        move_direction = "前进" if projected_distance > 0 else "后退"
                        adjustment_distance = min(
                            abs(projected_distance_meters), 0.3
                        )  # 限制单次微调距离

                        print(
                            f"沿当前航向进行微调: {move_direction} {adjustment_distance:.3f}米 (不转身)"
                        )

                        # 执行微调移动
                        if projected_distance > 0:
                            robot.go_forward(adjustment_distance)  # 前进
                        else:
                            robot.go_backward(adjustment_distance)  # 后退
                    else:
                        print(
                            f"投影距离较小 ({projected_distance:.2f} 像素)，不需要微调"
                        )

                    # 恢复原始设置
                    robot.enable_error_correction = original_error_correction
                    robot.speed_factor = original_speed_factor

                    # 更新位置并再次检查距离
                    gps_position = robot.get_current_position()
                    current_robot_position = map_transfor.webots_to_map(gps_position)
                    new_distance_to_goal = math.sqrt(
                        (current_robot_position[0] - goal[0]) ** 2
                        + (current_robot_position[1] - goal[1]) ** 2
                    )

                    logger.info(
                        f"微调后距离目标: {new_distance_to_goal:.2f} 像素 (之前: {distance_to_goal:.2f})"
                    )

                    # 检查微调是否有效
                    if new_distance_to_goal > distance_to_goal * 1.05:  # 距离增加超过5%
                        logger.warning(f"警告：微调后距离反而增大！")

                    # 微调后再次检查是否到达目标
                    if new_distance_to_goal <= 3:
                        logger.info(f"微调后到达目标! 距离: {new_distance_to_goal:.2f}")
                        send_robot_status(
                            current_robot_position,
                            f"Goal reached after adjustment! Distance: {new_distance_to_goal:.2f}",
                        )
                        goal_received = False
                        path = []
                        current_step_index = 0
    elif goal_received and not path:
        logger.info(f"No path found to {goal}. Waiting for new goal.")
        send_robot_status(current_robot_position, f"No path to {goal}.")
        goal_received = False

    time.sleep(0.1)
