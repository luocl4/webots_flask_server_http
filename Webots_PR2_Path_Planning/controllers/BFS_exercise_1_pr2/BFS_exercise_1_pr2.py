from utilities import MyCustomRobot
import json
import time
import requests
import math
from map_refiner import MapRefiner, inflate_obstacles  # 导入地图精细化模块

WEB_SERVER_URL = "http://127.0.0.1:5000/robot_command"

# 添加这个常量在ROBOT_START_POS附近
SCALE_FACTOR = 20
ROBOT_WIDTH = 0.8 * SCALE_FACTOR  # 机器人宽度
ROBOT_HEIGHT = 0.8 * SCALE_FACTOR  # 机器人高度

FINAL_APPROACH_DISTANCE = 4  # 接近目标的距离阈值(格子数)
original_start = (7, 1)  # 原始地图中的起始点
safety_radius = 1  # 安全膨胀半径，单位为精细化地图中的像素/格子 (允许小数点值)
goal = None
goal_yaw = None  # 添加目标偏航角变量，角度制，范围-180到180度

# 创建机器人实例 - 注意：只创建一个Robot实例
robot = MyCustomRobot(verbose=False)
robot.initialize_devices()
timestep = int(robot.getBasicTimeStep())

# 使用从Supervisor获得的位置信息替代
current_robot_position = None

# 原始世界地图
original_world_map = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 1, 0],
    [0, 1, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0]
]

# 精细化地图
map_refiner = MapRefiner(scale_factor=SCALE_FACTOR)
refined_world_map = map_refiner.refine_world_map(original_world_map)

# print(f"正在为障碍物添加 {safety_radius} 像素的安全边界...")
refined_world_map = inflate_obstacles(refined_world_map, safety_radius)
# print("障碍物膨胀完成，现在使用带有安全边界的地图进行路径规划")

# 转换世界地图参考点
start = map_refiner.convert_coordinates(original_start)
# print(f"原始起始点 {original_start} 在精细化地图中的坐标: {start}")

# 初始化机器人当前位置
current_robot_position = start

# 在机器人开始移动之前，调用收缩机械臂的方法
# 可以在这里立即调用，或者在收到第一个目标并计算出路径后调用
robot.retract_arms() # <<<<<<<<<<<<<<< 在这里调用，让机械臂在机器人开始规划和移动前收缩

def is_robot_position_valid(position, map_data=None):
    """
    检查考虑机器人大小后的位置是否有效
    机器人被视为中心在position，尺寸为ROBOT_WIDTH x ROBOT_HEIGHT的矩形
    优化：只检查机器人轮廓的四条边是否有障碍物，而不是整个边界框区域
    """
    map_data = refined_world_map if map_data is None else map_data
    
    # 计算机器人的边界框
    half_width = ROBOT_WIDTH / 2  # 使用浮点数除法
    half_height = ROBOT_HEIGHT / 2  # 使用浮点数除法
    
    # 确保位置坐标是整数
    position = (int(position[0]), int(position[1]))
    
    # 机器人的边界框顶点
    top_left = (position[0] - half_height, position[1] - half_width)
    top_right = (position[0] - half_height, position[1] + half_width)
    bottom_left = (position[0] + half_height, position[1] - half_width)
    # bottom_right = (position[0] + half_height, position[1] + half_width)
    
    # 地图尺寸
    map_height = len(map_data)
    map_width = len(map_data[0]) if map_height > 0 else 0
    
    # 计算用于检查的边界坐标（取整以确保覆盖所有格子）
    top_row = math.floor(top_left[0])
    bottom_row = math.ceil(bottom_left[0])
    left_col = math.floor(top_left[1])
    right_col = math.ceil(top_right[1])
    
    # 1. 检查边界是否超出地图范围
    if (top_row < 0 or bottom_row >= map_height or 
        left_col < 0 or right_col >= map_width):
        return False
    
    # 2. 只检查机器人的四条边界线
    # 检查上下边界
    for col in range(left_col, right_col + 1):
        if map_data[top_row][col] == 1:
            return False
        if map_data[bottom_row][col] == 1:
            return False
    
    # 检查左右边界
    for row in range(top_row + 1, bottom_row):  # +1避免重复检查角点
        if map_data[row][left_col] == 1:
            return False
        if map_data[row][right_col] == 1:
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
        (0, -1, "west"),   # 西
        (0, 1, "east"),     # 东
        (1, 0, "south")   # 南
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
    
    print(f"Moving {steps} steps {direction} from {current}, total distance: {total_distance:.2f}")
    print(f"距离最近障碍物: {obstacle_distance} 格, 速度系数: {safety_multiplier:.2f}")
    
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
        print(f"接近目标点 (剩余步数: {steps} <= {FINAL_APPROACH_DISTANCE})，关闭纠偏功能以提高稳定性")
    
    # 前进多步，使用自适应速度
    robot.set_movement_speed_factor(safety_multiplier)  # 设置速度系数
    robot.go_forward(total_distance)
    
    # 恢复默认设置
    robot.set_movement_speed_factor(1.0)  # 恢复默认速度
    
    if approaching_goal:
        # 恢复原始纠偏设置
        robot.enable_error_correction = original_error_correction
        print("恢复纠偏设置:", "开启" if original_error_correction else "关闭")


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
        "object_id": 1 if object_id is None else object_id  # 使用提供的ID或默认值
    }
    
    try:
        requests.post("http://127.0.0.1:5000/robot_status", json=payload)
    except requests.exceptions.ConnectionError:
        print("Could not connect to web server. Is it running?")
    except Exception as e:
        print(f"Error sending status: {e}")

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
        (1, 0),   # 南
        (0, -1),  # 西
        (0, 1),   # 东
        (-1, -1), # 西北
        (-1, 1),  # 东北
        (1, -1),  # 西南
        (1, 1)    # 东南
    ]
    
    map_height = len(map_data)
    map_width = len(map_data[0]) if map_height > 0 else 0
    
    min_distance = max_scan_distance
    
    for dir_row, dir_col in directions:
        for distance in range(1, max_scan_distance + 1):
            scan_row = int(position[0] + dir_row * distance)
            scan_col = int(position[1] + dir_col * distance)
            
            # 检查是否超出地图边界
            if scan_row < 0 or scan_row >= map_height or scan_col < 0 or scan_col >= map_width:
                min_distance = min(min_distance, distance)
                break
            
            # 检查是否遇到障碍物
            if map_data[scan_row][scan_col] == 1:
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
        print(error_message)
        # 立即向后端发送状态信息，通知路径规划失败
        error_info = {
            "status": "error",
            "message": error_message,
            "current_position": start,
            "is_reachable": False,
            "error_message": error_message,
            "path_planning_complete": True  # 明确标记路径规划已完成，但失败了
        }
        send_robot_status(start, error_info, object_id)
        return []
    
    if not is_robot_position_valid(goal, map_data):
        error_message = f"终点 {goal} 位置对机器人大小无效!"
        print(error_message)
        # 立即向后端发送状态信息，通知路径规划失败
        error_info = {
            "status": "error",
            "message": error_message,
            "current_position": start,
            "is_reachable": False,
            "error_message": error_message,
            "path_planning_complete": True  # 明确标记路径规划已完成，但失败了
        }
        send_robot_status(start, error_info, object_id)
        return []
    
    # 使用队列实现BFS
    from collections import deque
    queue = deque([(start, None, None)])  # (position, parent, direction)
    
    # 记录已访问的节点
    visited = {start: None}  # position: (parent, direction)
    
    nodes_explored = 0  # 统计探索的节点数
    
    while queue:
        current, parent, current_direction = queue.popleft()
        nodes_explored += 1
        
        # 到达目标
        if current == goal:
            print(f"BFS算法完成: 探索了 {nodes_explored} 个节点")
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
    
    # 检查是否找到路径
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
            "path_planning_complete": True  # 明确标记路径规划已完成，但失败了
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
    
    print(f"原始路径长度: {len(raw_path)} 步")
    
    # 直接简化路径 - 增强版，改进步数计算和路径合并
    path = []
    i = 0
    
    while i < len(raw_path) - 1:
        start_pos = raw_path[i]
        current_direction = direction_path[i+1] if i+1 < len(direction_path) else None
        
        if current_direction is None:
            i += 1
            continue
            
        # 计算连续相同方向的步数
        steps = 0
        j = i
        
        while j < len(raw_path) - 1 and j+1 < len(direction_path):
            if direction_path[j+1] == current_direction:
                steps += 1
                j += 1
            else:
                break
        
        # 计算实际距离而不是简单的步数转换
        # 注意：在高精度地图中，可能需要更精确的距离计算
        distance_in_cells = steps
        
        # 改进的路径简化逻辑 - 支持小距离精确移动
        # 计算精确的移动距离（以像素为单位，然后转换为实际步数）
        if distance_in_cells < SCALE_FACTOR:
            # 对于小距离，使用分数步数而不是强制为1
            # 将像素距离转换为实际的移动距离分数
            simplified_steps = distance_in_cells / SCALE_FACTOR
            # 设置最小移动距离，避免过小的移动
            if simplified_steps < 0.1:  # 小于0.1个网格单位的移动
                simplified_steps = 0.1
        else:
            # 计算简化步数，避免过度简化导致定位误差
            simplified_steps = distance_in_cells / SCALE_FACTOR  # 使用精确除法而不是四舍五入
            # 对长距离路径进行更细粒度的简化，避免一次走太远
            if simplified_steps > 10:
                # 将长路径分解为多个较短的部分，每部分不超过8步
                max_steps_per_segment = 8
                segments = []
                remaining_steps = simplified_steps
                current_segment_pos = start_pos
                
                while remaining_steps > 0:
                    segment_steps = min(remaining_steps, max_steps_per_segment)
                    segments.append((current_segment_pos, segment_steps, current_direction))
                    remaining_steps -= segment_steps
                    
                    # 更新下一段的起始位置 (估算，实际移动时会有所不同)
                    if current_direction == "north":
                        current_segment_pos = (current_segment_pos[0] - segment_steps * SCALE_FACTOR, current_segment_pos[1])
                    elif current_direction == "south":
                        current_segment_pos = (current_segment_pos[0] + segment_steps * SCALE_FACTOR, current_segment_pos[1])
                    elif current_direction == "west":
                        current_segment_pos = (current_segment_pos[0], current_segment_pos[1] - segment_steps * SCALE_FACTOR)
                    elif current_direction == "east":
                        current_segment_pos = (current_segment_pos[0], current_segment_pos[1] + segment_steps * SCALE_FACTOR)
                
                # 将分段添加到路径中
                path.extend(segments)
                i = j  # 移动到下一个不同方向的起始点
                continue
        
        # 添加到简化路径
        path.append((start_pos, simplified_steps, current_direction))
        i = j  # 移动到下一个不同方向的起始点
    
    print(f"路径规划: 原始路径 {len(raw_path)} 步 → 简化路径 {len(path)} 步")
    return path

def move_to_precise_pixel_position(current_pixel_pos, target_pixel_pos, scale_factor=SCALE_FACTOR):
    """
    精确移动到指定的像素位置，支持小数步长
    
    Args:
        current_pixel_pos: 当前像素位置 (row, col)
        target_pixel_pos: 目标像素位置 (row, col)
        scale_factor: 缩放因子
    
    Returns:
        bool: 是否成功移动
    """
    print(f"开始精确移动: 从 {current_pixel_pos} 到 {target_pixel_pos}")
    
    # 计算像素级的移动向量
    pixel_dx = target_pixel_pos[1] - current_pixel_pos[1]  # 列差(东西方向)
    pixel_dy = target_pixel_pos[0] - current_pixel_pos[0]  # 行差(南北方向)
    
    print(f"像素移动向量: dx={pixel_dx}, dy={pixel_dy}")
    
    # 将像素距离转换为实际移动距离(以DISTANCE_PER_GRID_CELL为单位)
    # 每个scale_factor像素对应1个DISTANCE_PER_GRID_CELL
    actual_dx_steps = pixel_dx / scale_factor  # 东西方向的步数(可以是小数)
    actual_dy_steps = pixel_dy / scale_factor  # 南北方向的步数(可以是小数)
    
    print(f"实际移动步数: dx_steps={actual_dx_steps:.3f}, dy_steps={actual_dy_steps:.3f}")
    
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
    return math.sqrt(dx*dx + dy*dy)

# 主循环:
current_robot_position = start
path = []
current_step_index = 0
goal_received = False

while robot.step(timestep) != -1:
    try:
        response = requests.get(WEB_SERVER_URL)
        if response.status_code == 200:
            command_data = response.json()
            if "goal" in command_data and command_data["goal"]:
                try:
                    new_goal = command_data.get("goal", [])
                    new_goal_yaw = command_data.get("goal_angle", 0.0)  # 获取目标偏航角，默认为0度
                    
                    if len(new_goal) >= 2:  # 确保目标坐标至少有两个元素
                        new_goal = (int((new_goal[0] + 0.5)*SCALE_FACTOR), int((new_goal[1] + 0.5)*SCALE_FACTOR))
                        if path:
                            print(f"当前路径: {path}")
                        if new_goal != goal or new_goal_yaw != goal_yaw:
                            goal = new_goal
                            goal_yaw = new_goal_yaw
                            print(f"收到新目标点: {goal}, 目标偏航角: {goal_yaw}度")
                            print("开始路径规划...")
                            time_start = time.time()
                            # 传递机器人ID给BFS_planning函数
                            current_object_id = command_data.get("object_id", 1)
                            path = BFS_planning(refined_world_map, current_robot_position, goal, current_object_id)
                            print("BFS路径规划完成，耗时: {:.2f} 秒".format(time.time() - time_start))
                            
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
                                    "path_planning_complete": True  # 明确标记路径规划已完成且成功
                                }
                                send_robot_status(current_robot_position, status_info)
                            else:
                                goal_received = False
                                goal = None
                                path = []
                    else:
                        print(f"警告: 收到无效的目标格式: {new_goal}")
                except (TypeError, IndexError) as e:
                    print(f"处理目标坐标时出错: {e}, 收到的数据: {command_data.get('goal')}")
    except requests.exceptions.ConnectionError:
        pass
    except json.JSONDecodeError:
        print("Invalid JSON response from web server.")
    except Exception as e:
        print(f"Error fetching command: {e}")

    if goal_received and path:
        # 计算当前位置与目标的距离
        distance_to_goal = math.sqrt((current_robot_position[0] - goal[0])**2 + 
                                   (current_robot_position[1] - goal[1])**2)
        
        # 当距离小于3像素时视为到达目标位置
        if distance_to_goal <= 3:
            print(f"到达目标位置! 距离目标: {distance_to_goal:.2f} (小于等于3像素)")
            
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
                "object_id": command_data.get("object_id", 1) if 'command_data' in locals() else 1,
                "total_segments": len(path),
                "current_segment": len(path),  # 当前段设为总段数，表示已完成
                "progress": 100.0,  # 进度100%
                "is_reachable": True,
                "is_completed": True
            }
            send_robot_status(current_robot_position, completion_info)
            goal_received = False
            path = []
            current_step_index = 0
            # robot.retract_arms()  # 可选：到达目标后收回机械臂
        else:
            # 如果路径已完成但尚未到达目标点，并且此时没有正在执行的微调，则进行微调
            if current_step_index >= len(path):
                print(f"路径已完成但尚未到达目标点，距离目标还有: {distance_to_goal:.2f} 像素")
                
                # 获取当前机器人的航向角度
                current_heading = robot.get_current_angle_from_imu()
                
                # 计算到目标的方向向量
                dx = goal[0] - current_robot_position[0]
                dy = goal[1] - current_robot_position[1]
                
                # 计算在当前航向上的投影距离（仅在当前朝向上前后移动，不转身）
                heading_vector_x = math.cos(current_heading)
                heading_vector_y = math.sin(current_heading)
                projected_distance = dx * heading_vector_x + dy * heading_vector_y
                projected_distance_meters = projected_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                
                # 改用像素级精确移动方法
                print(f"路径已完成但尚未到达目标点，使用像素级精确移动")
                print(f"当前位置: {current_robot_position}, 目标: {goal}")
                print(f"直线距离: {distance_to_goal:.2f} 像素")
                
                # 尝试使用精确移动到目标
                success = move_to_precise_pixel_position(current_robot_position, goal, SCALE_FACTOR)
                
                if success:
                    # 更新位置并验证
                    gps_position = robot.get_current_position()
                    current_robot_position = map_refiner.convert_coordinates(gps_position)
                    final_distance = calculate_pixel_distance(current_robot_position, goal)
                    
                    print(f"精确移动完成，最终距离: {final_distance:.2f} 像素")
                    
                    # 如果仍未到达且距离不太大，尝试使用utilities中的精确定位方法
                    if final_distance > 3 and final_distance <= 10:
                        print("尝试使用GPS基础的精确定位方法...")
                        try:
                            precise_success = robot.fine_tune_position_to_pixel(goal, SCALE_FACTOR, tolerance_pixels=3)
                            if precise_success:
                                print("GPS精确定位成功!")
                                gps_position = robot.get_current_position()
                                current_robot_position = map_refiner.convert_coordinates(gps_position)
                                final_distance = calculate_pixel_distance(current_robot_position, goal)
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
                            "message": f"已到达目标位置! 距离: {final_distance:.2f} 像素, 已调整到目标偏航角: {goal_yaw}度",
                            "object_id": command_data.get("object_id", 1) if 'command_data' in locals() else 1,
                            "total_segments": len(path),
                            "current_segment": len(path),  # 当前段设为总段数，表示已完成
                            "progress": 100.0,  # 进度100%
                            "is_reachable": True,
                            "is_completed": True
                        }
                        send_robot_status(current_robot_position, completion_info)
                        goal_received = False
                        path = []
                        current_step_index = 0
                    else:
                        print(f"精确移动后距离仍较大: {final_distance:.2f} 像素，重新规划路径")
                        # 重新规划路径
                        print(f"从当前位置 {current_robot_position} 重新规划到目标 {goal}")
                        new_path = BFS_planning(refined_world_map, current_robot_position, goal, 
                                              command_data.get("object_id", 1) if 'command_data' in locals() else 1)
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
                    # 重新规划路径
                
                # 恢复原始设置
                robot.enable_error_correction = original_error_correction
                robot.speed_factor = original_speed_factor
                
                # 如果微调后仍未到达目标，且距离仍大于3像素，则重新规划路径
                if goal_received and distance_to_goal > 3:
                    print(f"微调后仍未到达目标，距离: {distance_to_goal:.2f}，等待新指令或重新规划路径")
                    send_robot_status(current_robot_position, f"Still not at goal after adjustment. Distance: {distance_to_goal:.2f}")
                    
                    # 可选：重新规划路径
                    # print("尝试重新规划路径...")
                    # path = BFS_planning(refined_world_map, current_robot_position, goal)
                    # current_step_index = 0
            
            # 使用化简后的路径进行移动
            if current_step_index < len(path):
                start_pos, steps, direction = path[current_step_index]
                
                # 获取当前机器人的航向角度
                current_heading = robot.get_current_angle_from_imu()
                
                # 计算到目标的方向向量
                dx = goal[0] - current_robot_position[0]
                dy = goal[1] - current_robot_position[1]
                
                # 计算在当前航向上的投影距离
                heading_vector_x = math.cos(current_heading)
                heading_vector_y = math.sin(current_heading)
                projected_distance = dx * heading_vector_x + dy * heading_vector_y
            
            # 使用化简后的路径进行移动
            if current_step_index < len(path):
                start_pos, steps, direction = path[current_step_index]

                print(f"Executing step {current_step_index}: {steps} steps {direction} from {start_pos}")
                move_from_to_multi_steps(current_robot_position, steps, direction)

                # 使用robot的GPS获取当前位置
                try:
                    gps_position = robot.get_current_position()
                    current_robot_position = map_refiner.convert_coordinates(gps_position)
                    
                    # 打印更详细的位置信息
                    print(f"当前位置 - GPS: ({gps_position[0]:.3f}, {gps_position[1]:.3f}), 地图: {current_robot_position}")
                    
                    # 计算与目标的距离
                    if goal:
                        distance_to_goal = math.sqrt((current_robot_position[0] - goal[0])**2 + 
                                                   (current_robot_position[1] - goal[1])**2)
                        print(f"距离目标还有: {distance_to_goal:.2f} 像素 ({distance_to_goal/SCALE_FACTOR:.2f} 格)")
                except Exception as e:
                    print(f"获取位置时出错: {e}")
                    # 如果出错，使用最后一个已知位置
                    print(f"使用最后一个已知位置: {current_robot_position}")
                
                current_step_index += 1
                # 更新状态，包含总段数和当前段索引，用于进度展示
                progress_info = {
                    "status": "移动中",
                    "message": f"正在沿路径移动，段 {current_step_index}/{len(path)}",
                    "object_id": command_data.get("object_id", 1) if 'command_data' in locals() else 1,
                    "total_segments": len(path),
                    "current_segment": current_step_index,
                    "progress": round(current_step_index / len(path) * 100, 1),  # 百分比进度
                    "is_reachable": True
                }
                send_robot_status(current_robot_position, progress_info)
            else:
                print("Simplified path completed, checking goal...")
                # 计算当前位置与目标的距离
                distance_to_goal = math.sqrt((current_robot_position[0] - goal[0])**2 + 
                                           (current_robot_position[1] - goal[1])**2)
                
                # 当距离小于3像素时视为到达目标
                if distance_to_goal <= 3:
                    print(f"Goal reached after simplified path traversal. 距离目标: {distance_to_goal:.2f} (小于等于3)")
                    send_robot_status(current_robot_position, f"Goal reached! 距离: {distance_to_goal:.2f}")
                    goal_received = False
                    path = []
                    current_step_index = 0
                else:
                    print(f"Path completed but goal not reached. 距离目标: {distance_to_goal:.2f} (大于3)，进行微调...")
                    send_robot_status(current_robot_position, f"Distance: {distance_to_goal:.2f}，performing fine adjustment")
                    
                    # 获取当前机器人的航向角度
                    current_heading = robot.get_current_angle_from_imu()
                    
                    # 计算到目标的方向向量
                    dx_actual = goal[0] - current_robot_position[0]
                    dy_actual = goal[1] - current_robot_position[1]
                    
                    # 计算到目标的实际距离
                    actual_distance = math.sqrt(dx_actual**2 + dy_actual**2)
                    actual_distance_meters = actual_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                    
                    # 计算在当前航向上的投影距离（仅在当前朝向上前后移动，不转身）
                    heading_vector_x = math.cos(current_heading)
                    heading_vector_y = math.sin(current_heading)
                    projected_distance = dx_actual * heading_vector_x + dy_actual * heading_vector_y
                    projected_distance_meters = projected_distance / SCALE_FACTOR * robot.DISTANCE_PER_GRID_CELL
                    
                    print(f"当前位置: {current_robot_position}, 目标: {goal}")
                    print(f"直线距离: {actual_distance:.2f} 像素 ({actual_distance_meters:.3f}米)")
                    print(f"当前航向投影距离: {projected_distance:.2f} 像素 ({projected_distance_meters:.3f}米)")
                    
                    # 保存原始设置
                    original_error_correction = robot.enable_error_correction
                    original_speed_factor = robot.speed_factor
                    
                    # 关闭误差纠偏，降低速度进行精确微调
                    robot.enable_error_correction = False
                    robot.speed_factor = 0.3  # 低速微调
                    
                    # 仅在当前朝向上进行前后移动微调，不转身
                    if abs(projected_distance) > 1.0:  # 投影距离大于1像素才进行微调
                        move_direction = "前进" if projected_distance > 0 else "后退"
                        adjustment_distance = min(abs(projected_distance_meters), 0.3)  # 限制单次微调距离
                        
                        print(f"沿当前航向进行微调: {move_direction} {adjustment_distance:.3f}米 (不转身)")
                        
                        # 执行微调移动
                        if projected_distance > 0:
                            robot.go_forward(adjustment_distance)  # 前进
                        else:
                            robot.go_backward(adjustment_distance)  # 后退
                    else:
                        print(f"投影距离较小 ({projected_distance:.2f} 像素)，不需要微调")
                    
                    # 恢复原始设置
                    robot.enable_error_correction = original_error_correction
                    robot.speed_factor = original_speed_factor
                    
                    # 更新位置并再次检查距离
                    gps_position = robot.get_current_position()
                    current_robot_position = map_refiner.convert_coordinates(gps_position)
                    new_distance_to_goal = math.sqrt((current_robot_position[0] - goal[0])**2 + 
                                                  (current_robot_position[1] - goal[1])**2)
                    
                    print(f"微调后距离目标: {new_distance_to_goal:.2f} 像素 (之前: {distance_to_goal:.2f})")
                    
                    # 检查微调是否有效
                    if new_distance_to_goal > distance_to_goal * 1.05:  # 距离增加超过5%
                        print(f"警告：微调后距离反而增大！")
                    
                    # 微调后再次检查是否到达目标
                    if new_distance_to_goal <= 3:
                        print(f"微调后到达目标! 距离: {new_distance_to_goal:.2f}")
                        send_robot_status(current_robot_position, f"Goal reached after adjustment! Distance: {new_distance_to_goal:.2f}")
                        goal_received = False
                        path = []
                        current_step_index = 0
    elif goal_received and not path:
        print(f"No path found to {goal}. Waiting for new goal.")
        send_robot_status(current_robot_position, f"No path to {goal}.")
        goal_received = False
    
    time.sleep(0.1)
