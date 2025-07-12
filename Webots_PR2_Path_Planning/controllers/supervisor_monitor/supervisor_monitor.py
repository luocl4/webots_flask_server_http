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
import threading
import http.server
import socketserver

# Configuration for Web communication
WEB_SERVER_URL = "http://127.0.0.1:5000/world_status"

supervisor = Supervisor()
robot_node = supervisor.getFromDef("PR2_ROBOT")
wordl_path = supervisor.getWorldPath()
floor = supervisor.getFromDef("floor")
timestep = int(supervisor.getBasicTimeStep())


def safe_float(value, default=0.0):
    """安全地将值转换为浮点数"""
    try:
        if isinstance(value, (int, float)):
            return float(value)
        elif isinstance(value, str):
            return float(value)
        else:
            return default
    except (ValueError, TypeError):
        return default


# NODES_TO_MONITOR = [
#     {
#         'id': 1,
#         'name': "PR2_ROBOT",
#     },
#     {
#         'id': 2,
#         'name': "Box_1",
#     },
#     {
#         'id': 3,
#         'name': "Box_2",
#     },
#     {
#         'id': 4,
#         'name': "Box_3",
#     },
#     {
#         'id': 5,
#         'name': "Box_4",
#     },
#     {
#         'id': 6,
#         'name': "Box_5",
#     },
#     {
#         'id': 7,
#         'name': "Box_6",
#     },
#     {
#         'id': 8,
#         'name': "Box_7",
#     },
#     {
#         'id': 9,
#         'name': "Box_8",
#     },
#     {
#         'id': 10,
#         'name': "Box_9",
#     },
#     {
#         'id': 11,
#         'name': "Box_10",
#     },
#     {
#         'id': 12,
#         'name': "Box_11",
#     },
#     {
#         'id': 13,
#         'name': "Box_12",
#     },
#     {
#         'id': 14,
#         'name': "Box_13",
#     },
#     {
#         'id': 15,
#         'name': "goodbox",
#     },
#     {
#         'id': 16,
#         'name': "conver",
#     },
#     {
#         'id': 17,
#         'name': "Obstacle_Box_1",
#     },
#     {
#         'id': 18,
#         'name': "Obstacle_Box_2"
#     }
# ]

NODES_TO_MONITOR = [
    {
        'id': 1,
        'name': "PR2_ROBOT",
    },
    {'id': 2, 'name': "cardboard_box(1)"},
    {'id': 3, 'name': "cardboard_box(2)"},
    {'id': 4, 'name': "cardboard_box(3)"},
    {'id': 5, 'name': "cardboard_box(4)"},
    {'id': 6, 'name': "cardboard_box(5)"},
    {'id': 7, 'name': "cardboard_box(6)"},
    {'id': 8, 'name': "cardboard_box(7)"},
    {'id': 9, 'name': "cardboard_box(8)"},
    {'id': 10, 'name': "PlasticCrate0"},
    {'id': 11, 'name': "PlasticCrate1"},
    {'id': 12, 'name': "PlasticCrate2"},
    {'id': 13, 'name': "PlasticCrate3"},
]

# 存储上一次发送的节点数据，用于判断是否移动。
# 键现在是节点的 DEF 名称，而不是 Webots 内部的数字 ID。
last_node_data = {}

# 存储每个 Solid 对象的位置偏移量（第一次计算后缓存）
solid_position_offsets = {}

# 存储每个 Solid 对象的描述信息缓存（第一次读取后缓存）
solid_describe_cache = {}

# 移动阈值（位置和旋转）
POSITION_THRESHOLD = 0.005  # 5 毫米
ROTATION_THRESHOLD = 0.5  # 0.5 度


def send_world_status(nodes_status_list):
    """Sends the current world status (node properties) to the Flask server."""
    payload = {"timestamp": time.time(), "nodes": nodes_status_list}
    try:
        requests.post(WEB_SERVER_URL, json=payload)
    except requests.exceptions.ConnectionError:
        print("Supervisor WARNING: Could not connect to web server. Is it running?")
    except Exception as e:
        print(f"Supervisor ERROR: Failed to send world status: {e}")


def get_callable_methods(obj):
    """获取对象的所有可调用方法"""
    methods = []
    for attr_name in dir(obj):
        try:
            attr = getattr(obj, attr_name)
            if callable(attr) and not attr_name.startswith('_'):
                methods.append(attr_name)
        except Exception:
            continue
    return methods


def get_node_properties(node_webots_object):
    """
    Extracts relevant properties from a Webots node object.
    Returns a dictionary.
    """
    if not node_webots_object:
        return None

    Type_name = node_webots_object.getTypeName()

    # 获取节点名称（即 DEF 名称）
    node_def_name = node_webots_object.getDef()
    if not node_def_name:  # 确保 DEF 存在
        print(f"Supervisor WARNING: Node found without DEF name. Skipping.")
        return None

    position = node_webots_object.getPosition()

    # 获取旋转矩阵
    rotation_matrix = node_webots_object.getOrientation()

    # 处理万向锁问题 (gimbal lock)
    if abs(rotation_matrix[6]) >= 0.99999:  # 当 R31 接近 ±1 时发生万向锁
        # 万向锁发生，roll 和 yaw 合并为一个自由度
        roll_rad = 0.0  # 设置 roll 为 0
        # 特殊情况处理
        if rotation_matrix[6] > 0:  # R31 = 1
            pitch_rad = -math.pi / 2  # -90度
            yaw_rad = -math.atan2(rotation_matrix[1], rotation_matrix[4])
        else:  # R31 = -1
            pitch_rad = math.pi / 2  # 90度
            yaw_rad = math.atan2(rotation_matrix[1], rotation_matrix[4])
    else:
        # 正常情况，没有万向锁
        pitch_rad = -math.asin(rotation_matrix[6])  # 从 R31 提取 pitch
        cos_pitch = math.cos(pitch_rad)

        # 计算 roll
        roll_rad = math.atan2(
            rotation_matrix[7] / cos_pitch, rotation_matrix[8] / cos_pitch
        )

        # 计算 yaw
        yaw_rad = math.atan2(
            rotation_matrix[3] / cos_pitch, rotation_matrix[0] / cos_pitch
        )

    # 转换为角度
    roll_degrees = math.degrees(roll_rad)
    pitch_degrees = math.degrees(pitch_rad)
    yaw_degrees = math.degrees(yaw_rad)

    # 确保角度在 -180 到 180 度范围内
    roll_degrees = ((roll_degrees + 180) % 360) - 180
    pitch_degrees = ((pitch_degrees + 180) % 360) - 180
    yaw_degrees = ((yaw_degrees + 180) % 360) - 180

    if Type_name == "Solid":
        # 使用缓存的描述信息，避免重复读取字段
        if node_def_name not in solid_describe_cache:
            # 第一次读取，获取并缓存描述信息
            result = get_describe_from_solid(node_webots_object)
            solid_describe_cache[node_def_name] = result
            print(f"Supervisor INFO: 缓存 {node_def_name} 的描述信息")
        else:
            # 使用缓存的描述信息
            result = solid_describe_cache[node_def_name]
        if result:
            color_value = result['color']
            size_value = result['size'].split('/') if result['size'] else [0, 0, 0]
            describe_value = result['description']
            ability_value = (
                result['ability_code'].split('/') if result['ability_code'] else []
            )
            origin_pos_str = result['origin_pos']
            obj_type = result['obj_type']

            # 处理位置偏移量（只在第一次计算）
            if node_def_name not in solid_position_offsets:
                # 第一次读取，计算偏移量
                if origin_pos_str:
                    try:
                        origin_pos = [float(x) for x in origin_pos_str.split('/')]
                        if len(origin_pos) >= 3:
                            # 计算偏移量 = 当前位置 - 原始位置
                            offset = [
                                position[0] - origin_pos[0],
                                position[1] - origin_pos[1],
                                position[2] - origin_pos[2],
                            ]
                            solid_position_offsets[node_def_name] = offset
                            print(
                                f"Supervisor INFO: 计算 {node_def_name} 位置偏移量: {offset}"
                            )
                        else:
                            solid_position_offsets[node_def_name] = [0, 0, 0]
                    except (ValueError, IndexError) as e:
                        print(
                            f"Supervisor WARNING: 解析 {node_def_name} 原始位置失败: {e}"
                        )
                        solid_position_offsets[node_def_name] = [0, 0, 0]
                else:
                    solid_position_offsets[node_def_name] = [0, 0, 0]

            # 应用偏移量修正位置
            offset = solid_position_offsets[node_def_name]
            corrected_position = [
                position[0] - offset[0],
                position[1] - offset[1],
                position[2] - offset[2],
            ]
            position = corrected_position
        else:
            size_value = [0, 0, 0]
            describe_value = "无法获取描述"
            ability_value = []
    elif Type_name == "PlasticFruitBox" or Type_name == 'CardboardBox':
        # 使用缓存的描述信息，避免重复读取字段
        if node_def_name not in solid_describe_cache:
            # 第一次读取，获取并缓存描述信息
            result = get_describe_from_solid(node_webots_object)
            solid_describe_cache[node_def_name] = result
            print(f"Supervisor INFO: 缓存 {node_def_name} 的描述信息")
        else:
            # 使用缓存的描述信息
            result = solid_describe_cache[node_def_name]
        print(result)
        if result:
            color_value = result['color']
            size_value = result['size'].split('/') if result['size'] else [0, 0, 0]
            describe_value = result['description']
            ability_value = (
                result['ability_code'].split('/') if result['ability_code'] else []
            )
            obj_type = result['obj_type']
        else:
            size_value = [0, 0, 0]
            describe_value = "无法获取描述"
            ability_value = []
    elif Type_name == "urdf_arm":
        describe_value, ability_value, size_value = get_obj_describe(
            node_webots_object
        )  # 获取 describe 字段
        if ability_value:
            ability_value = parse_ability(ability_value)
        obj_type = '1'
    else:
        size_value = get_obj_size(node_webots_object)  # 获取 size 字段
        describe_value, ability_value, _ = get_obj_describe(
            node_webots_object
        )  # 获取 describe 字段
        if ability_value:
            ability_value = parse_ability(ability_value)
        obj_type = '1'

    return {
        "name": node_def_name,  # 使用 DEF 名称作为标识符
        "position": [round(p, 3) for p in position],  # 保留三位小数，方便阅读
        "rotation_degrees": {
            "roll": round(roll_degrees, 2),
            "pitch": round(pitch_degrees, 2),
            "yaw": round(yaw_degrees, 2),
        },
        "size": size_value,
        "describe": describe_value,
        "ability": ability_value,
        "obj_type": obj_type,
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
                return describe_value, None, None
            else:
                describe_field = node_webots_object.getField('customData')
                if describe_field:
                    describe_value = describe_field.getSFString()
                    custom_data = json.loads(describe_value)
                    describe = custom_data.get("describe", "未知描述")
                    ability = custom_data.get("ability", "未知能力")
                    obj_size = custom_data.get("size", "0.0__0.0__0.0").split('__')
                    return describe, ability, obj_size
                else:
                    return None, None, None
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


def parse_field_string_detailed(field_str):
    parts = field_str.split('//')

    while len(parts) < 5:
        parts.append('')

    result = {
        'size': parts[0] if len(parts) > 0 else '',
        'description': parts[1] if len(parts) > 1 else '',
        'color': parts[2] if len(parts) > 2 else '',
        'ability_code': parts[3] if len(parts) > 3 else '',
        'origin_pos': parts[4] if len(parts) > 4 else '',
        'obj_type': parts[5] if len(parts) > 5 else '',
    }

    return result


def get_describe_from_solid(node_webots_object):
    try:
        all_field = node_webots_object.getField('description')
        if all_field:
            field_str = all_field.getSFString()
            if field_str:
                parts = parse_field_string_detailed(field_str)
                return parts
            else:
                return None
        else:
            return None
    except Exception as e:
        print(f"获取ability字段时出错: {e}")


def has_moved(current_props, last_props):
    """
    Compares current properties with last properties to determine if the node has moved significantly.
    When a node is first seen (no last_props), it is considered *not* moving initially.
    """
    if not last_props:
        return False  # 第一次见到，默认为未移动

    # 检查位置变化
    current_pos = current_props["position"]
    last_pos = last_props["position"]
    pos_diff = math.sqrt(
        (current_pos[0] - last_pos[0]) ** 2
        + (current_pos[1] - last_pos[1]) ** 2
        + (current_pos[2] - last_pos[2]) ** 2
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
send_interval = 1.0  # 每秒发送一次状态更新

while supervisor.step(timestep) != -1:
    current_time = supervisor.getTime()

    if current_time - last_send_time >= send_interval:
        nodes_status = []
        for mon_mode in NODES_TO_MONITOR:
            node_name = mon_mode.get('name')
            node = supervisor.getFromDef(node_name)  # 获取节点对象
            if node:
                properties = get_node_properties(node)
                if properties:
                    node_def_name = properties["name"]  # 获取节点的 DEF 名称

                    moved = has_moved(properties, last_node_data.get(node_def_name, {}))
                    properties["is_moving"] = moved  # 添加 is_moving 字段
                    properties["id"] = mon_mode.get('id')

                    nodes_status.append(properties)
                    last_node_data[node_def_name] = (
                        properties  # 更新上次数据，使用 DEF 名称作为键
                    )
                else:
                    print(
                        f"Supervisor WARNING: Could not get properties for node '{node_name}'. It might not have a DEF name or its type is not supported."
                    )
            else:
                print(
                    f"Supervisor WARNING: Node '{node_name}' not found in the world. Please check your .wbt file and NODES_TO_MONITOR list."
                )

        if nodes_status:
            send_world_status(nodes_status)
            # 在终端打印当前状态（简洁版，避免刷屏）
            for node_s in nodes_status:
                # move_status = "移动" if node_s["is_moving"] else "静止"
                print(
                    f"Supervisor INFO: 节点 '{node_s['name']}': 位置={node_s['position']}, 偏航角={node_s['rotation_degrees']['yaw']:.2f}°"
                )

        last_send_time = current_time
