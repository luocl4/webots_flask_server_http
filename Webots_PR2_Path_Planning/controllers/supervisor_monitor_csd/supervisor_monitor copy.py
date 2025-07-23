# supervisor_monitor.py (修改后的完整文件)

from controller import Supervisor
import time
import requests
import json
import math
import base64
import io
from PIL import Image
import numpy as np

# Configuration for Web communication
WEB_SERVER_URL = "http://127.0.0.1:5000/world_status"
CAMERA_STATUS_URL = "http://127.0.0.1:5000/camera_status"

supervisor = Supervisor()
floor = supervisor.getFromDef("floor")
timestep = int(supervisor.getBasicTimeStep())

# 获取相机设备
camera_rgb = None
camera_dpt = None

def init_devices():
    global camera_rgb, camera_dpt
    
    # 获取RGB相机
    camera_rgb = supervisor.getDevice('camera_rgb')
    if camera_rgb:
        camera_rgb.enable(50)
        print("RGB相机初始化成功")
    else:
        print("警告: 找不到RGB相机")
        
    # 获取深度相机
    camera_dpt = supervisor.getDevice('camera_dpt')
    if camera_dpt:
        camera_dpt.enable(50)
        print("深度相机初始化成功")
    else:
        print("警告: 找不到深度相机")

# 调用初始化函数
init_devices()

NODES_TO_MONITOR = [
    {
        'id': 1,
        'name': "PR2_ROBOT",
    },
    {
        'id': 2,
        'name': "OBSTACLE_BOX_1",
    },
    {
        'id': 3,
        'name': "OBSTACLE_BOX_2",
    },
    {
        'id': 4,
        'name': "OBSTACLE_BOX_3",
    },
    {
        'id': 5,
        'name': "OBSTACLE_BOX_4",
    },
    {
        'id': 6,
        'name': "OBSTACLE_BOX_5",
    }
]

# 存储上一次发送的节点数据，用于判断是否移动。
# 键现在是节点的 DEF 名称，而不是 Webots 内部的数字 ID。
last_node_data = {}

# 移动阈值（位置和旋转）
POSITION_THRESHOLD = 0.005  # 5 毫米
ROTATION_THRESHOLD = 0.5    # 0.5 度

def send_world_status(nodes_status_list):
    """ Sends the current world status (node properties) to the Flask server. """
    payload = {
        "timestamp": time.time(),
        "nodes": nodes_status_list
    }
    try:
        requests.post(WEB_SERVER_URL, json=payload)
    except requests.exceptions.ConnectionError:
        print("Supervisor WARNING: Could not connect to web server. Is it running?")
    except Exception as e:
        print(f"Supervisor ERROR: Failed to send world status: {e}")

def get_node_properties(node_webots_object):
    """
    Extracts relevant properties from a Webots node object.
    Returns a dictionary.
    """
    if not node_webots_object:
        return None
    
    # 获取节点名称（即 DEF 名称）
    node_def_name = node_webots_object.getDef()
    if not node_def_name: # 确保 DEF 存在
        print(f"Supervisor WARNING: Node found without DEF name. Skipping.")
        return None

    position = node_webots_object.getPosition()
    # Webots 的 getOrientation 返回的是一个 3x3 旋转矩阵，需要转换为欧拉角
    # 或者直接使用 getPose() 然后提取旋转
    # 对于简单的 Yaw 角，getOrientation() 的 [7] 元素（sin(theta_z)）和 [0] (cos(theta_z))可以用来计算
    # 或者 Webots API 也提供了更直接的方法来获取轴角或四元数
    # 为了简化，我们只提取 Yaw 角，并确保 Roll 和 Pitch 默认为 0
    
    # 获取旋转矩阵
    rotation_matrix = node_webots_object.getOrientation()
    
    # 从旋转矩阵计算欧拉角 (roll, pitch, yaw)
    # 注意：Webots 矩阵是行优先存储的，所以 indices:
    # [0] R11 (xx)  [1] R12 (xy)  [2] R13 (xz)
    # [3] R21 (yx)  [4] R22 (yy)  [5] R23 (yz)
    # [6] R31 (zx)  [7] R32 (zy)  [8] R33 (zz)
    
    # 基于旋转矩阵从三轴旋转矩阵中提取欧拉角 (roll-pitch-yaw, XYZ顺序)
    # 这是一个标准的欧拉角提取算法
    
    # 处理万向锁问题 (gimbal lock)
    if abs(rotation_matrix[6]) >= 0.99999:  # 当 R31 接近 ±1 时发生万向锁
        # 万向锁发生，roll 和 yaw 合并为一个自由度
        roll_rad = 0.0  # 设置 roll 为 0
        # 特殊情况处理
        if rotation_matrix[6] > 0:  # R31 = 1
            pitch_rad = -math.pi/2  # -90度
            yaw_rad = -math.atan2(rotation_matrix[1], rotation_matrix[4])
        else:  # R31 = -1
            pitch_rad = math.pi/2  # 90度
            yaw_rad = math.atan2(rotation_matrix[1], rotation_matrix[4])
    else:
        # 正常情况，没有万向锁
        pitch_rad = -math.asin(rotation_matrix[6])  # 从 R31 提取 pitch
        cos_pitch = math.cos(pitch_rad)
        
        # 计算 roll
        roll_rad = math.atan2(rotation_matrix[7]/cos_pitch, rotation_matrix[8]/cos_pitch)
        
        # 计算 yaw
        yaw_rad = math.atan2(rotation_matrix[3]/cos_pitch, rotation_matrix[0]/cos_pitch)
    
    # 转换为角度
    roll_degrees = math.degrees(roll_rad)
    pitch_degrees = math.degrees(pitch_rad)
    yaw_degrees = math.degrees(yaw_rad)
    
    # 确保角度在 -180 到 180 度范围内
    roll_degrees = ((roll_degrees + 180) % 360) - 180
    pitch_degrees = ((pitch_degrees + 180) % 360) - 180
    yaw_degrees = ((yaw_degrees + 180) % 360) - 180

    size_value = get_obj_size(node_webots_object) # 获取 size 字段
    describe_value, ability_value = get_obj_describe(node_webots_object) # 获取 describe 字段
    if ability_value:
        ability_value = parse_ability(ability_value)

    return {
        "name": node_def_name, # 使用 DEF 名称作为标识符
        "position": [round(p, 3) for p in position], # 保留三位小数，方便阅读
        "rotation_degrees": {
            "roll": round(roll_degrees, 2),
            "pitch": round(pitch_degrees, 2),
            "yaw": round(yaw_degrees, 2)
        },
        "size" : size_value,
        "describe": describe_value,
        "ability": ability_value
    }

def get_obj_size(node_webots_object):
    try:
        if hasattr(node_webots_object, 'getField'):
            type_name = node_webots_object.getField('name').getSFString()
            if 'wooden box' in type_name:
                size_field = node_webots_object.getField('size')
                if size_field:
                    size_value = size_field.getSFVec3f()
                    return size_value
                else:
                    return None
            else:
                return None
    except Exception as e:
        print(f"获取size字段时出错: {e}")
    
def get_obj_describe(node_webots_object):
    try:
        if hasattr(node_webots_object, 'getField'):
            describe_field = node_webots_object.getField('describe')
            if describe_field:
                describe_value = describe_field.getSFString()
                # print(f"获取到描述字段: {describe_value}")
                return describe_value, None
            else:
                describe_field = node_webots_object.getField('customData')
                if describe_field:
                    describe_value = describe_field.getSFString()
                    custom_data = json.loads(describe_value)
                    describe = custom_data.get("describe", "未知描述")
                    ability = custom_data.get("ability", "未知能力")
                    print(f"获取到自定义数据字段: {describe_value}")
                    return describe, ability
                else:
                    return None, None
    except Exception as e:
        print(f"获取size字段时出错: {e}")
    
def get_obj_ability(node_webots_object):
    """
    获取物体的能力（如是否可移动、是否可交互等）。
    目前假设所有物体都具有相同的能力。
    """
    try:
        if hasattr(node_webots_object, 'getField'):
            ability_field = node_webots_object.getField('ability')
            if ability_field:
                ability_value = ability_field.getSFString()
                return ability_value
            else:
                return None
    except Exception as e:
        print(f"获取ability字段时出错: {e}")
        
def parse_ability(ability_str):
    """
    解析物体的能力字符串，返回一个字典。
    假设能力字符串是以逗号分隔的键值对形式。
    """
    if not ability_str:
        return None
    
    return ability_str.split('__')

def has_moved(current_props, last_props):
    """
    Compares current properties with last properties to determine if the node has moved significantly.
    When a node is first seen (no last_props), it is considered *not* moving initially.
    """
    if not last_props:
        return False # 第一次见到，默认为未移动

    # 检查位置变化
    current_pos = current_props["position"]
    last_pos = last_props["position"]
    pos_diff = math.sqrt(
        (current_pos[0] - last_pos[0])**2 +
        (current_pos[1] - last_pos[1])**2 +
        (current_pos[2] - last_pos[2])**2
    )
    if pos_diff > POSITION_THRESHOLD:
        return True

    # 检查旋转变化 (只看 Yaw 角)
    current_yaw = current_props["rotation_degrees"]["yaw"]
    last_yaw = last_props["rotation_degrees"]["yaw"]
    yaw_diff = abs(current_yaw - last_yaw)
    # 考虑到角度循环，处理 360 度跨越
    if yaw_diff > 180:
        yaw_diff = 360 - yaw_diff
    if yaw_diff > ROTATION_THRESHOLD:
        return True

    return False

last_send_time = 0
send_interval = 1.0 # 每秒发送一次状态更新

# 优化深度图发送逻辑
def send_camera_data():
    # 处理RGB图像
    if camera_rgb and camera_rgb.getImage():
        try:
            image_data = camera_rgb.getImage()
            # 将Webots相机图像转换为PIL图像
            pil_image = Image.frombytes('RGBA', (camera_rgb.getWidth(), camera_rgb.getHeight()), image_data)
            # 将图像转换为base64字符串
            img_byte_arr = io.BytesIO()
            pil_image.save(img_byte_arr, format='PNG')
            img_byte_arr = img_byte_arr.getvalue()
            encoded_image = base64.b64encode(img_byte_arr).decode('utf-8')
            
            requests.post(CAMERA_STATUS_URL, json={'image': encoded_image})
        except requests.exceptions.RequestException as e:
            print(f"Error sending RGB camera data: {e}")
        except Exception as e:
            print(f"Error processing RGB camera data: {e}")

    # 处理深度图像
    if camera_dpt and camera_dpt.getRangeImage():
        try:
            depth_data = camera_dpt.getRangeImage()
            width = camera_dpt.getWidth()
            height = camera_dpt.getHeight()
            
            # 检查深度数据是否有效
            if depth_data and len(depth_data) == width * height:
                # 将深度数据转换为numpy数组并重塑为2D
                depth_array = np.array(depth_data, dtype=np.float32)
                depth_2d = depth_array.reshape((height, width))
                
                # 处理深度数据中的NaN和无穷大值
                depth_2d = np.nan_to_num(depth_2d, nan=0.0, posinf=10.0, neginf=0.0)
                
                # 将深度数据归一化到0-255范围
                depth_normalized = (depth_2d * 255 / 10.0).clip(0, 255).astype(np.uint8)
                
                # 创建PIL图像
                depth_image = Image.fromarray(depth_normalized, mode='L')
                
                # 将图像转换为base64字符串
                img_byte_arr = io.BytesIO()
                depth_image.save(img_byte_arr, format='PNG')
                img_byte_arr = img_byte_arr.getvalue()
                encoded_depth_image = base64.b64encode(img_byte_arr).decode('utf-8')
                
                requests.post(CAMERA_STATUS_URL, json={'depth_image': encoded_depth_image})
            else:
                print(f"Invalid depth data: expected {width * height} elements, got {len(depth_data) if depth_data else 0}")
                
        except requests.exceptions.RequestException as e:
            print(f"Error sending depth camera data: {e}")
        except Exception as e:
            print(f"Error processing depth camera data: {e}")

while supervisor.step(timestep) != -1:
    current_time = supervisor.getTime()

    if current_time - last_send_time >= send_interval:
        nodes_status = []
        for mon_mode in NODES_TO_MONITOR:
            node_name = mon_mode.get('name')
            node = supervisor.getFromDef(node_name) # 获取节点对象
            if node:
                properties = get_node_properties(node)
                if properties:
                    node_def_name = properties["name"] # 获取节点的 DEF 名称

                    moved = has_moved(properties, last_node_data.get(node_def_name, {}))
                    properties["is_moving"] = moved # 添加 is_moving 字段
                    properties["id"] = mon_mode.get('id')

                    nodes_status.append(properties)
                    last_node_data[node_def_name] = properties # 更新上次数据，使用 DEF 名称作为键
                else:
                    print(f"Supervisor WARNING: Could not get properties for node '{node_name}'. It might not have a DEF name or its type is not supported.")
            else:
                print(f"Supervisor WARNING: Node '{node_name}' not found in the world. Please check your .wbt file and NODES_TO_MONITOR list.")

        if nodes_status:
            send_world_status(nodes_status)
            # 在终端打印当前状态（简洁版，避免刷屏）
            for node_s in nodes_status:
                move_status = "移动" if node_s["is_moving"] else "静止"
                print(f"Supervisor INFO: 节点 '{node_s['name']}' ({move_status}): 位置={node_s['position']}, 偏航角={node_s['rotation_degrees']['yaw']:.2f}°")
        
        last_send_time = current_time

    # send_camera_data()  # 发送相机数据