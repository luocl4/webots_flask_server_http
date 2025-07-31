from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
from collections import deque
import subprocess
import signal
import os
import sys
import threading
import time
import json
import base64
import io
from datetime import datetime
import wbt_parser
from logger import logger
import logging
import math
import numpy as np
from Webots_PR2_Path_Planning.controllers.new_ik.ik_solve_pr2 import ArmIk


# 添加控制器目录到系统路径
controller_path = os.path.join(
    os.path.dirname(__file__),
    "Webots_PR2_Path_Planning",
    "controllers",
    "BFS_exercise_1_pr2",
)
sys.path.append(controller_path)
controller_path = os.path.join(
    os.path.dirname(__file__),
    "Webots_PR2_Path_Planning",
    "controllers",
    "BFS_exercise_1_kuavo",
)
sys.path.append(controller_path)
controller_path = os.path.join(
    os.path.dirname(__file__),
    "Webots_PR2_Path_Planning",
    "controllers",
    "BFS_exercise_1_kuavo_lb",
)
sys.path.append(controller_path)
controller_path = os.path.join(
    os.path.dirname(__file__), "Webots_capture", "controllers", "arm_control_leju"
)
sys.path.append(controller_path)
app = Flask(__name__)
CORS(app)

# 配置Flask的werkzeug日志级别，减少频繁接口的日志输出
werkzeug_logger = logging.getLogger("werkzeug")
werkzeug_logger.setLevel(logging.WARNING)
sl_status = {}


# 创建自定义过滤器，过滤掉特定接口的日志
class FilterFrequentEndpoints(logging.Filter):
    def filter(self, record):
        if hasattr(record, "getMessage"):
            message = record.getMessage()
            if "/robot_command" in message or "/world_status" in message:
                return False
        return True


# 将过滤器应用到werkzeug日志器
werkzeug_logger.addFilter(FilterFrequentEndpoints())

# ================= Webots进程控制相关变量 =================
webots_process = None
webots_process_lock = threading.Lock()
current_loaded_scene = None  # 当前已加载的场景信息

# ================= 机器人状态管理相关变量 =================
# 为每个机器人ID维护单独的状态
# 状态码说明：1-未移动，2-移动中，3-移动结束
robot_goals = {}
capture_robot_goals = {}
pick_robot_goals = {}
place_robot_goals = {}
robot_stop_flag = False
robot_status_dict = {}
result_pick = False
fail_pick = False
fail_place = False
result_place = False
arm_go_pos_robot_goal = {}
fail_arm_go_pos = False
current_left_pos = []
current_right_pos = []
current_left_rpy = []
current_right_rpy = []
robot_status = {"robot_position": (0, 0), "status": ""}  # 存储机器人的位置和状态信息

ability_dict = {
    "left_place": "左手放置",
    "right_place": "右手放置",
    "both_place": "双手放置",
    "move": "移动",
    "turn": "转身",
    "left_wipe": "左手擦拭",
    "right_wipe": "右手擦拭",
    "squat": "下蹲",
    "left_grab": "左手抓取",
    "right_grab": "右手抓取",
    "both_grab": "双手抓取",
    "bend": "弯腰",
    "identify": "识别",
}

command_queue = deque()
SCENE_CONFIGS = {}
request_response_log = deque(maxlen=10)
supervisor_world_status_log = deque(maxlen=10)
latest_camera_image = None
latest_depth_image = None
log_dir = "log"


# ================= 工具函数 =================
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


def load_scene_configs():
    """加载场景配置文件"""
    global SCENE_CONFIGS
    config_file = os.path.join(
        os.path.dirname(__file__), "configs", "scene_configs.json"
    )
    try:
        with open(config_file, "r", encoding="utf-8") as f:
            SCENE_CONFIGS = json.load(f)
        logger.info(f"场景配置文件加载成功: {config_file}")
        return SCENE_CONFIGS
    except FileNotFoundError:
        logger.warning(f"场景配置文件未找到: {config_file}，使用默认配置")
    except json.JSONDecodeError as e:
        logger.error(f"场景配置文件JSON格式错误: {e}，使用默认配置")
    except Exception as e:
        logger.error(f"加载场景配置文件时出错: {e}，使用默认配置")

    return {
        "1": {
            "id": "1",
            "name": "Path Planning Scene",
            "version": "2.1.0",
            "world_file": "path_planning.wbt",
            "corners": [[4, 4], [-4, -4], [4, -4], [-4, 4]],
            "working_dir": "./Webots_PR2_Path_Planning/worlds",
            "label_point": [{"p1": {"x": 0, "y": 0, "z": 0}}],
            "description": "路径规划场景",
        },
    }


def kill_all_webots_processes():
    """杀死所有Webots和xvfb相关进程"""
    global current_loaded_scene
    try:
        # 要杀死的进程类型列表
        process_types = [("webots", "Webots"), ("xvfb", "Xvfb")]

        killed_any = False

        for process_pattern, process_name in process_types:
            # 查找相关进程
            result = subprocess.run(
                ["pgrep", "-f", process_pattern], capture_output=True, text=True
            )

            if result.stdout.strip():
                pids = result.stdout.strip().split("\n")
                logger.info(f"找到 {len(pids)} 个 {process_name} 进程")

                # 先发送SIGTERM信号
                for pid in pids:
                    try:
                        pid = int(pid.strip())
                        os.kill(pid, signal.SIGTERM)
                        logger.info(f"发送SIGTERM到{process_name}进程 {pid}")
                        killed_any = True
                        time.sleep(0.1)
                    except (ValueError, ProcessLookupError):
                        continue

                # 等待一秒后检查是否还有残留进程
                time.sleep(1)
                result = subprocess.run(
                    ["pgrep", "-f", process_pattern], capture_output=True, text=True
                )

                if result.stdout.strip():
                    remaining_pids = result.stdout.strip().split("\n")
                    logger.warning(
                        f"发现 {len(remaining_pids)} 个残留的{process_name}进程，强制杀死"
                    )

                    # 强制杀死残留进程
                    for pid in remaining_pids:
                        try:
                            pid = int(pid.strip())
                            os.kill(pid, signal.SIGKILL)
                            logger.warning(f"强制杀死{process_name}进程 {pid}")
                        except (ValueError, ProcessLookupError):
                            continue
                else:
                    logger.info(f"所有{process_name}进程已成功终止")
            else:
                logger.info(f"未找到运行中的{process_name}进程")

        # 清理当前场景信息
        current_loaded_scene = None
        if killed_any:
            logger.info("已清理当前场景信息")

        return True
    except Exception as e:
        logger.error(f"杀死进程时出错: {e}", exc_info=True)
        return False


def convert_to_list(value):
    """将NumPy数组转换为列表，其他类型保持不变"""
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def cleanup_temp_wbt_files():
    """清理临时wbt文件"""
    try:
        # 获取worlds目录路径
        wbt_dir_path = os.path.join(
            os.path.dirname(__file__), "Webots_PR2_Path_Planning", "worlds"
        )

        if not os.path.exists(wbt_dir_path):
            logger.warning(f"worlds目录不存在: {wbt_dir_path}")
            return

        # 查找所有temp_前缀的wbt文件
        temp_files = []
        for filename in os.listdir(wbt_dir_path):
            if filename.startswith("temp_") and filename.endswith(".wbt"):
                temp_files.append(filename)

        if temp_files:
            logger.info(f"发现{len(temp_files)}个临时wbt文件，正在清理...")
            for filename in temp_files:
                file_path = os.path.join(wbt_dir_path, filename)
                try:
                    os.remove(file_path)
                    logger.info(f"已删除临时文件: {filename}")
                except OSError as e:
                    logger.warning(f"删除临时文件失败 {filename}: {e}")
            logger.info("临时wbt文件清理完成")
        else:
            logger.info("未发现需要清理的临时wbt文件")

    except Exception as e:
        logger.error(f"清理临时wbt文件时出错: {e}", exc_info=True)


# ================= 世界状态管理路由 =================
@app.route("/world_status", methods=["POST"])
def update_world_status():
    """接收世界状态数据"""
    data = request.get_json()
    if "nodes" in data and "timestamp" in data:
        supervisor_world_status_log.append(
            {
                "event_type": "world_status_update",
                "data": data["nodes"],
                "message": "World status updated by supervisor.",
            }
        )

        for node in data["nodes"]:
            if node.get("is_moving"):
                logger.info(
                    f"节点 '{node['name']}' (正在移动): 位置={node['position']}, 偏航角={node['rotation_degrees']['yaw']:.2f}°"
                )

        return jsonify({"message": "World status updated"}), 200
    else:
        logger.warning(f"Received invalid world status update: {data}")
        request_response_log.append(
            {
                "event_type": "world_status_update_failure",
                "request_data": data,
                "message": "Invalid data for world status update.",
                "error": "Missing 'nodes' or 'timestamp'",
            }
        )
        return jsonify({"error": "Invalid world status data"}), 400


@app.route("/robot_command", methods=["GET"])
def get_robot_command():
    """获取机器人命令"""
    global robot_goals, robot_stop_flag, capture_robot_goals, pick_robot_goals, place_robot_goals, arm_go_pos_robot_goal
    command_to_send = {}

    if robot_stop_flag:
        robot_stop_flag = False
        return jsonify(None), 200

    if capture_robot_goals:
        command_to_send = capture_robot_goals
        command_to_send["source"] = "robot_capture"
    elif pick_robot_goals:
        command_to_send = pick_robot_goals
        command_to_send["source"] = "robot_pick"
    elif place_robot_goals:
        command_to_send = place_robot_goals
        command_to_send["source"] = "robot_place"
    elif arm_go_pos_robot_goal:
        command_to_send = arm_go_pos_robot_goal
        command_to_send["source"] = "arm_go_pos"
    else:
        # 找出第一个处于移动状态的机器人
        for object_id, goal_state in robot_goals.items():
            if goal_state.get("status") == 2 and "goal_pos" in goal_state:
                goal_pos = goal_state.get("goal_pos", [])
                goal_angle = goal_state.get("goal_angle", 0.0)  # 如果没有角度，默认为0
                start_time = goal_state.get(
                    "start_time", datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                )

                if len(goal_pos) == 2:  # 确保至少有目标位置
                    command_to_send = {
                        "goal": [goal_pos[0], goal_pos[1]],
                        "goal_angle": goal_angle,
                        "object_id": object_id,  # 添加object_id信息
                        "goal_timestamp": start_time,  # 添加时间戳用于区分不同的goal请求
                        "source": "robot_goals",  # 标记来源
                    }
                    break
                elif len(goal_pos) == 0:  # 如果目标位置为空
                    command_to_send = {
                        "goal": [],
                        "goal_angle": goal_angle,
                        "object_id": object_id,  # 添加object_id信息
                        "goal_timestamp": start_time,  # 添加时间戳用于区分不同的goal请求
                        "source": "robot_goals",  # 标记来源
                    }
                    break
    # if command_to_send != {}:
    #     print(command_to_send)

    request_response_log.append(
        {
            "event_type": "get_robot_command",
            "response_data": command_to_send,
            "message": "Webots controller fetched command.",
        }
    )

    return jsonify(command_to_send), 200


@app.route("/set_sl_status", methods=["POST"])
def update_sl_status():
    """更新机器人状态"""
    global sl_status
    try:
        # 尝试不同的方式获取数据
        data = None

        # 方式1：尝试获取JSON数据
        if request.is_json:
            data = request.get_json()
        # 方式2：尝试获取表单数据
        elif request.form:
            data = request.form.to_dict()
            # 尝试解析JSON字符串
            if "data" in data:
                try:
                    import json

                    data = json.loads(data["data"])
                except:
                    pass
        # 方式3：尝试获取原始数据并解析为JSON
        elif request.data:
            try:
                import json

                data = json.loads(request.data.decode("utf-8"))
            except:
                pass

        if not data:
            return jsonify({"code": 400, "message": "参数为空或格式错误"}), 400

        required_fields = ["id", "object_name", "state_name", "command"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        name_data = data["object_name"]
        scene_id = str(data["id"])
        if isinstance(name_data, list):
            try:
                obj_ids = [str(item) for item in name_data]
            except (ValueError, TypeError):
                return (
                    jsonify({"code": 400, "message": "object_name列表中包含无效值"}),
                    400,
                )
        else:
            return jsonify({"code": 400, "message": "object_name必须是列表"}), 400

        sl_status = {
            "object_names": obj_ids,
            "state_name": data["state_name"],
            "command": data["command"],
        }

    except Exception as e:
        logger.error(f"更新机器人状态失败: {e}", exc_info=True)
        return jsonify({"code": 500, "message": str(e)}), 500

    return jsonify({"code": 200, "message": "Status updated"}), 200


@app.route("/get_sl_status", methods=["GET"])
def get_sl_status():
    """获取机器人状态"""
    global sl_status

    try:
        # 检查sl_status是否为空或未初始化
        if not sl_status or sl_status == {}:
            return jsonify({"code": 404, "message": "状态未设置"}), 404

        # 验证必要字段是否存在
        if "object_names" not in sl_status:
            return jsonify({"code": 500, "message": "缺少object_names字段"}), 500

        if "state_name" not in sl_status:
            return jsonify({"code": 500, "message": "缺少state_name字段"}), 500

        # 验证object_names是否为列表
        if not isinstance(sl_status["object_names"], list):
            return jsonify({"code": 500, "message": "object_names必须是列表"}), 500

        # 构建返回数据
        if len(sl_status["object_names"]) == 0:
            sl_info = [
                {
                    "object_name": "world",
                    "state_name": sl_status["state_name"],
                    "command": sl_status["command"],
                }
            ]
        else:
            sl_info = [
                {
                    "object_name": obj_name,
                    "state_name": sl_status["state_name"],
                    "command": sl_status["command"],
                }
                for obj_name in sl_status["object_names"]
            ]

        sl_status = {}

        return jsonify(sl_info), 200

    except Exception as e:
        logger.error(f"获取机器人状态时出错: {e}", exc_info=True)
        return jsonify({"code": 500, "message": f"服务器错误: {str(e)}"}), 500


@app.route("/robot_status", methods=["POST"])
def update_robot_status():
    """更新机器人状态"""
    global current_left_rpy, current_right_rpy, current_left_pos, current_right_pos, result_pick, result_place, robot_status, robot_goals, robot_status_dict, capture_robot_goals, pick_robot_goals, place_robot_goals, fail_pick, fail_place, fail_arm_go_pos, arm_go_pos_robot_goal
    data = request.get_json()
    # 获取status字典（如果存在）
    status_dict = data.get("status")
    if status_dict and isinstance(status_dict, dict):
        # 检查task和status字段是否存在
        task = status_dict.get("task")
        status_value = status_dict.get("status")
        # print("status_dict", status_dict)
        if task == "pick":
            result_pick = True
            if status_value == "fail":
                fail_pick = True
            else:
                fail_pick = False
                res = status_dict.get("res")
                # 确保current_left_pos和current_right_pos是列表
                current_left_pos = status_dict.get("current_left_pos")
                current_right_pos = status_dict.get("current_right_pos")
                current_left_rpy = status_dict.get("current_left_rpy")
                current_right_rpy = status_dict.get("current_right_rpy")
                # logger.info("current_left_rpy", current_left_rpy)
                # logger.info("current_right_rpy", current_right_rpy)
        elif task == "place":
            result_place = True
            if status_value == "fail":
                fail_place = True
            else:
                fail_place = False
                res = status_dict.get("res")
                # 确保current_left_pos和current_right_pos是列表
                current_left_pos = status_dict.get("current_left_pos")
                current_right_pos = status_dict.get("current_right_pos")
                current_left_rpy = status_dict.get("current_left_rpy")
                current_right_rpy = status_dict.get("current_right_rpy")
                # logger.info("current_left_rpy", current_left_rpy)
                # logger.info("current_right_rpy", current_right_rpy)
        elif task == "arm_to_go":
            if status_value == "fail":
                fail_arm_go_pos = True
            else:
                fail_arm_go_pos = False
                # 确保current_left_pos和current_right_pos是列表
                current_left_pos = status_dict.get("current_left_pos")
                current_right_pos = status_dict.get("current_right_pos")
                current_left_rpy = status_dict.get("current_left_rpy")
                current_right_rpy = status_dict.get("current_right_rpy")

        if (
            task == "capture"
            or task == "pick"
            or task == "place"
            or task == "arm_to_go"
        ):
            print(data)
            capture_robot_goals = {}
            pick_robot_goals = {}
            place_robot_goals = {}
            robot_goals = {}
            arm_go_pos_robot_goal = {}
            return jsonify({"message": "Status updated"}), 200

    # 检查数据有效性
    if not data or not isinstance(data, dict):
        logger.warning(f"接收到无效的机器人状态更新: {data}")
        request_response_log.append(
            {
                "event_type": "robot_status_update_failure",
                "request_data": data,
                "message": "Invalid data format for robot status update.",
            }
        )
        return jsonify({"error": "Invalid status data format"}), 400
    # 处理新格式的状态数据 - 支持新的对象结构
    print(data.get("status"))
    if isinstance(data.get("status"), dict):
        # 新格式：状态是一个包含详细信息的字典
        status_dict = data["status"]
        current_position = (
            tuple(data["robot_position"]) if "robot_position" in data else (0, 0)
        )
        current_status = status_dict.get("message", "未知状态")
        object_id = int(status_dict.get("object_id", 1))
        is_reachable = status_dict.get("is_reachable", True)
        is_completed = status_dict.get("is_completed", False)
        total_segments = status_dict.get("total_segments", 0)
        current_segment = status_dict.get("current_segment", 0)
        progress = status_dict.get("progress", 0)
        error_message = status_dict.get("message", "")  # 获取错误信息
        path_planning_complete = status_dict.get(
            "path_planning_complete", False
        )  # 获取路径规划完成标志

        # 更新全局状态
        robot_status["robot_position"] = current_position
        robot_status["status"] = current_status

        # 确保对象ID存在于字典中
        if object_id not in robot_goals:
            robot_goals[object_id] = {"status": 1, "goal_pos": None}

        # 更新机器人目标状态
        # 更新可达性标志和路径规划完成标志
        robot_goals[object_id]["is_reachable"] = is_reachable
        robot_goals[object_id]["path_planning_complete"] = path_planning_complete

        # 如果是初次接收到路径规划结果，记录日志
        if path_planning_complete and not robot_goals[object_id].get(
            "path_planning_logged", False
        ):
            robot_goals[object_id]["path_planning_logged"] = True
            if is_reachable:
                logger.info(
                    f"路径规划已完成且成功: 机器人(ID:{object_id}), 总段数:{total_segments}"
                )
            else:
                logger.warning(
                    f"路径规划已完成但失败: 机器人(ID:{object_id}), 错误:{error_message}"
                )

        # 如果目标不可达，记录错误信息
        if not is_reachable:
            robot_goals[object_id]["error_message"] = error_message
            logger.warning(f"目标不可达: 机器人(ID:{object_id}), 错误:{error_message}")

        # 更新进度信息
        if total_segments > 0:
            robot_goals[object_id]["total_segments"] = total_segments
            robot_goals[object_id]["current_segment"] = current_segment
            robot_goals[object_id]["process"] = f"{current_segment}/{total_segments}"
            robot_goals[object_id]["progress"] = progress

        # 更新状态码
        if not is_reachable:
            # 目标不可达，状态设为4(错误)
            robot_goals[object_id]["status"] = 4
            robot_status_dict[object_id] = 4
            logger.info(f"机器人 {object_id} 目标不可达: {current_status}")
        elif is_completed:
            # 已到达目标，状态设为3(完成)
            robot_goals[object_id]["status"] = 3
            robot_status_dict[object_id] = 3
            logger.info(f"机器人 {object_id} 到达目标: {current_status}")
        else:
            # 仍在移动中，状态保持为2(移动中)
            robot_goals[object_id]["status"] = 2
            robot_status_dict[object_id] = 2

        logger.info(
            f"机器人状态更新(新格式): ID={object_id}, 位置={current_position}, 状态='{current_status}', 进度={current_segment}/{total_segments}"
        )

    # 处理旧格式的状态数据 - 向后兼容
    elif "robot_position" in data and "status" in data:
        current_position = tuple(data["robot_position"])
        current_status = data["status"]
        object_id = int(data.get("object_id", 1))

        # 更新全局状态
        robot_status["robot_position"] = current_position
        robot_status["status"] = current_status

        # 确保对象ID存在于字典中
        if object_id not in robot_goals:
            robot_goals[object_id] = {"status": 1, "goal_pos": None}

        # 检查是否包含"Goal reached"或"到达目标"信息
        if isinstance(current_status, str) and (
            "Goal reached" in current_status or "到达目标" in current_status
        ):
            # 移动完成，更新为状态3
            robot_goals[object_id]["status"] = 3
            robot_status_dict[object_id] = 3
            robot_goals[object_id]["process"] = "5/5"  # 假设总共5段路径
            logger.info(f"机器人 {object_id} 到达目标 (旧格式): {current_status}")

        logger.info(
            f"机器人状态更新(旧格式): ID={object_id}, 位置={current_position}, 状态='{current_status}'"
        )

    else:
        logger.warning(f"接收到格式不完整的机器人状态更新: {data}")
        request_response_log.append(
            {
                "event_type": "robot_status_update_failure",
                "request_data": data,
                "message": "Invalid data for robot status update.",
                "error": "Missing required fields",
            }
        )
        return jsonify({"error": "Missing required fields"}), 400

    # 记录状态更新
    request_response_log.append(
        {
            "event_type": "robot_status_update_success",
            "request_data": data,
            "message": "Robot status updated.",
        }
    )

    return jsonify({"message": "Status updated"}), 200


# ================= 相机图像管理路由 =================
@app.route("/camera_status", methods=["POST"])
def update_camera_status():
    """更新相机状态"""
    global latest_camera_image, latest_depth_image
    try:
        data = request.get_json()
        if "image" in data:
            latest_camera_image = data["image"]
        if "depth_image" in data:
            latest_depth_image = data["depth_image"]
        return jsonify({"status": "success"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/get_camera_image", methods=["GET"])
def get_camera_image():
    """获取RGB相机图像"""
    global latest_camera_image
    try:
        if latest_camera_image is None:
            return jsonify({"error": "No camera image available"}), 404

        image_data = base64.b64decode(latest_camera_image)
        img_io = io.BytesIO(image_data)

        return send_file(img_io, mimetype="image/png")
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/get_depth_image", methods=["GET"])
def get_depth_image():
    """获取深度相机图像"""
    global latest_depth_image
    try:
        if latest_depth_image is None:
            return jsonify({"error": "No depth image available"}), 404

        image_data = base64.b64decode(latest_depth_image)
        img_io = io.BytesIO(image_data)

        return send_file(img_io, mimetype="image/png")
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# ================= 日志保存函数 =================
def save_main_log_on_exit():
    """保存主日志文件"""
    log_filename = os.path.join(log_dir, "flask_server_activity_log.json")
    try:
        # 将 deque 转换为 list 以便 JSON 序列化
        log_data = list(request_response_log)
        with open(log_filename, "w", encoding="utf-8") as f:
            json.dump(log_data, f, indent=4, ensure_ascii=False)
        logger.info(f"主 Flask 服务器活动日志已保存至 {log_filename}")
    except Exception as e:
        logger.error(f"保存主日志文件失败: {e}", exc_info=True)


def save_supervisor_log_on_exit():
    """保存Supervisor世界状态日志文件"""
    supervisor_log_filename = os.path.join(log_dir, "supervisor_world_status_log.json")
    try:
        # 将 deque 转换为 list 以便 JSON 序列化
        supervisor_log_data = list(supervisor_world_status_log)
        with open(supervisor_log_filename, "w", encoding="utf-8") as f:
            json.dump(supervisor_log_data, f, indent=4, ensure_ascii=False)
        logger.info(f"Supervisor 世界状态日志已保存至 {supervisor_log_filename}")
    except Exception as e:
        logger.error(f"保存 Supervisor 日志文件失败: {e}", exc_info=True)


# ================ 服务器启动和关闭处理 =================
@app.route("/api/v1/scenes", methods=["GET"])
def get_available_scenes():
    """获取可用的场景列表"""
    # 获取name参数用于过滤
    name_filter = request.args.get("name")

    scenes_data = []

    for i, (scene_id, config) in enumerate(SCENE_CONFIGS.items(), 1):
        # 如果提供了name参数，只返回匹配的场景
        if name_filter and config["name"] != name_filter:
            continue

        world_file_path = os.path.join(config["working_dir"], config["world_file"])
        size = "0B"

        if os.path.exists(world_file_path):
            try:
                file_size = os.path.getsize(world_file_path)
                if file_size < 1024:
                    size = f"{file_size}B"
                elif file_size < 1024 * 1024:
                    size = f"{file_size // 1024}KB"
                else:
                    size = f"{file_size // (1024 * 1024)}MB"

            except OSError:
                size = "Unknown"

        scene_data = {
            "id": i,
            "name": config["name"],
            "version": config["version"],
            "size": size,
            "description": config["description"],
            "scene_id": config["id"],
            "coordinate": config["corners"],
            "label_point": config["label_point"],
        }
        scenes_data.append(scene_data)

    # 如果提供了name参数但没有找到匹配的场景
    if name_filter and not scenes_data:
        return (
            jsonify(
                {
                    "code": 404,
                    "message": f"未找到名称为 '{name_filter}' 的场景",
                    "data": [],
                }
            ),
            404,
        )

    return jsonify({"code": 200, "message": "success", "data": scenes_data}), 200


@app.route("/api/v1/scene/load", methods=["GET"])
def start_webots():
    """启动Webots进程"""
    global webots_process, current_loaded_scene, supervisor_world_status_log
    # print(current_loaded_scene)
    with webots_process_lock:
        try:
            data = {}
            if request.is_json:
                data = request.get_json() or {}
            elif request.form:
                data = request.form.to_dict()
            elif request.args:
                data = request.args.to_dict()

            scene_id = data.get("id")

            if not scene_id:
                return (
                    jsonify(
                        {"code": 400, "message": "缺少必需参数 'id'，即场景的编号"}
                    ),
                    400,
                )

            if scene_id not in SCENE_CONFIGS:
                available_ids = list(SCENE_CONFIGS.keys())
                return (
                    jsonify(
                        {
                            "code": 404,
                            "message": f"场景ID '{scene_id}' 不存在，可用场景: {', '.join(available_ids)}",
                        }
                    ),
                    404,
                )

            scene_config = SCENE_CONFIGS[scene_id]

            # 清理之前的进程
            logger.info("启动前清理所有Webots进程...")
            kill_all_webots_processes()
            webots_process = None
            current_loaded_scene = None

            # 清空supervisor_world_status_log
            supervisor_world_status_log = []

            world_file = scene_config["world_file"]
            working_dir = scene_config["working_dir"]
            description = scene_config["description"]
            scene_display_name = scene_config["name"]
            scene_version = scene_config["version"]

            logger.info(f"选择场景: {scene_id} - {scene_display_name} v{scene_version}")

            cmd = [
                "webots",
                "--mode=realtime",
                "--minimize",
                world_file,
                "--stream",
            ]

            webots_process = subprocess.Popen(
                cmd,
                cwd=working_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True,
            )

            # 更新当前已加载的场景信息
            real_id = scene_config["id"]
            current_loaded_scene = {
                "id": real_id,
                "name": scene_display_name,
                "version": scene_version,
                "world_file": world_file,
                "description": description,
                "pid": webots_process.pid,
                "loaded_at": datetime.now().isoformat(),
            }

            logger.info(
                f"Webots进程启动成功，PID: {webots_process.pid}, 场景: {scene_display_name}, 场景ID: {real_id}, 版本: {scene_version}"
            )

            # 等待supervisor更新世界状态
            logger.info("等待Supervisor更新世界状态...")

            # 设置超时时间（秒）
            timeout = 600
            start_time = time.time()

            # 等待supervisor_world_status_log更新或超时
            # print(supervisor_world_status_log)
            while (
                not supervisor_world_status_log and time.time() - start_time < timeout
            ):
                time.sleep(0.5)

            # print(supervisor_world_status_log)
            if supervisor_world_status_log:
                logger.info("Supervisor已更新世界状态，场景加载完成")
                return (
                    jsonify(
                        {
                            "code": 200,
                            "message": "success",
                            "data": {
                                "scene_id": real_id,
                                "scene_name": scene_display_name,
                                "pid": webots_process.pid,
                                "status": "started",
                            },
                        }
                    ),
                    200,
                )
            else:
                logger.warning("等待Supervisor更新世界状态超时，但Webots进程已启动")
                return (
                    jsonify(
                        {
                            "code": 202,
                            "message": "Webots进程已启动，但未检测到Supervisor更新世界状态",
                            "data": {
                                "scene_id": real_id,
                                "scene_name": scene_display_name,
                                "pid": webots_process.pid,
                                "status": "started_waiting_supervisor",
                            },
                        }
                    ),
                    202,
                )

        except Exception as e:
            error_msg = f"启动Webots进程失败: {str(e)}"
            logger.error(error_msg, exc_info=True)

            return jsonify({"code": 500, "message": error_msg}), 500


@app.route("/api/v1/scene/objects", methods=["GET"])
def get_world_status():
    """获取当前世界状态数据 - 所有对象列表"""
    global supervisor_world_status_log
    if not supervisor_world_status_log:
        return (
            jsonify(
                {
                    "code": 404,
                    "message": "暂无世界状态数据",
                    "data": {"total_count": 0, "list": []},
                }
            ),
            404,
        )

    try:
        # 获取最新的世界状态数据
        latest_log_entry = supervisor_world_status_log[-1]
        nodes_data = latest_log_entry.get("data", [])
        # print("nodes_data", nodes_data)
        # 转换为标准格式
        formatted_list = []

        for i, node in enumerate(nodes_data, 1):
            node_id = node.get("id", i)
            node_name = node.get("name", "Unknown")
            position = node.get("position", [0, 0, 0])
            yaw_angle = node.get("rotation_degrees").get("yaw")
            size = node.get("size", [0, 0, 0])
            ability_code = node.get("ability", None)
            ability_list = []
            if ability_code:
                for code in ability_code:
                    if code in ability_dict:
                        ability_list.append(
                            {"ability_code": code, "ability_name": ability_dict[code]}
                        )
            if not size:
                size = [0, 0, 0]
            describe = node.get("describe", "该物体暂无描述字段")

            if not isinstance(position, list) or len(position) < 3:
                position = [0, 0, 0]
            if not isinstance(size, list) or len(size) < 3:
                size = [0, 0, 0]

            formatted_node = {
                "id": node_id,
                "name": node_name,
                "ability": ability_list,
                "describe": describe,
                "obj_type": node.get("obj_type"),
                "position": {
                    "x": f"{safe_float(position[0]):.2f}",
                    "y": f"{safe_float(position[1]):.2f}",
                    "z": f"{safe_float(position[2]):.2f}",
                },
                "rotation": f"{safe_float(yaw_angle):.2f}",
                "size": {
                    "length": f"{safe_float(size[0]):.2f}",
                    "width": f"{safe_float(size[1]):.2f}",
                    "high": f"{safe_float(size[2]):.2f}",
                },
            }

            formatted_list.append(formatted_node)

        return (
            jsonify(
                {
                    "code": 200,
                    "message": "success",
                    "data": {
                        "total_count": len(formatted_list),
                        "list": formatted_list,
                    },
                }
            ),
            200,
        )

    except Exception as e:
        return jsonify({"code": 500, "message": f"获取世界状态数据失败: {str(e)}"}), 500


@app.route("/api/v1/scene/object", methods=["GET"])
def get_single_object():
    """获取单个物体的详细信息"""
    global supervisor_world_status_log, current_loaded_scene

    # 获取参数
    scene_id = request.args.get("id")
    object_id = request.args.get("object_id")

    # 参数验证
    if not scene_id:
        return jsonify({"code": 400, "message": "缺少必需参数 'id'"}), 400

    if not object_id:
        return jsonify({"code": 400, "message": "缺少必需参数 'object_id'"}), 400

    # 验证场景ID是否为当前已加载的场景
    if not current_loaded_scene or current_loaded_scene.get("id") != scene_id:
        current_scene = current_loaded_scene.get("id") if current_loaded_scene else "无"
        return (
            jsonify(
                {
                    "code": 409,
                    "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'",
                }
            ),
            409,
        )

    # 检查是否有世界状态数据
    if not supervisor_world_status_log:
        return jsonify({"code": 404, "message": "暂无世界状态数据"}), 404

    try:
        try:
            target_object_id = int(object_id)
        except ValueError:
            return (
                jsonify(
                    {
                        "code": 400,
                        "message": f"无效的object_id: '{object_id}'，必须是整数",
                    }
                ),
                400,
            )

        # 获取最新的世界状态数据
        latest_log_entry = supervisor_world_status_log[-1]
        nodes_data = latest_log_entry.get("data", [])

        # 查找目标物体
        target_object = None
        for i, node in enumerate(nodes_data, 1):
            if i == target_object_id:
                target_object = node
                break

        if not target_object:
            return (
                jsonify({"code": 404, "message": f"未找到ID为 '{object_id}' 的物体"}),
                404,
            )

        node_name = node.get("name", "Unknown")
        position = node.get("position", [0, 0, 0])
        size = node.get("size", [0, 0, 0])
        ability_code = node.get("ability", None)
        ability_list = []
        yaw_angle = node.get("rotation_degrees").get("yaw")
        if ability_code:
            for code in ability_code:
                if code in ability_dict:
                    ability_list.append(
                        {"ability_code": code, "ability_name": ability_dict[code]}
                    )
        if not size:
            size = [0, 0, 0]
        describe = node.get("describe", "该物体暂无描述字段")

        if not isinstance(position, list) or len(position) < 3:
            position = [0, 0, 0]
        if not isinstance(size, list) or len(size) < 3:
            size = [0, 0, 0]

        object_info = {
            "id": target_object_id,
            "name": node_name,
            "obj_type": node.get("obj_type"),
            "ability": ability_list,
            "describe": describe,
            "rotation": f"{safe_float(yaw_angle):.2f}",
            "position": {
                "x": f"{safe_float(position[0]):.2f}",
                "y": f"{safe_float(position[1]):.2f}",
                "z": f"{safe_float(position[2]):.2f}",
            },
            "size": {
                "length": f"{safe_float(size[0]):.2f}",
                "width": f"{safe_float(size[1]):.2f}",
                "high": f"{safe_float(size[2]):.2f}",
            },
        }

        return jsonify({"code": 200, "message": "success", "data": object_info}), 200

    except Exception as e:
        return jsonify({"code": 500, "message": f"获取物体信息失败: {str(e)}"}), 500

    except Exception as e:
        return jsonify({"code": 500, "message": f"获取世界状态数据失败: {str(e)}"}), 500


@app.route("/api/v1/scene/coordinate", methods=["GET"])
def track_moving_object():
    """获取单个物体的详细信息"""
    global supervisor_world_status_log, current_loaded_scene

    # 获取参数
    scene_id = request.args.get("id")
    object_id = request.args.get("object_id")

    # 参数验证
    if not scene_id:
        return jsonify({"code": 400, "message": "缺少必需参数 'id'"}), 400

    if not object_id:
        return jsonify({"code": 400, "message": "缺少必需参数 'object_id'"}), 400

    # 验证场景ID是否为当前已加载的场景
    if not current_loaded_scene or current_loaded_scene.get("id") != scene_id:
        current_scene = current_loaded_scene.get("id") if current_loaded_scene else "无"
        return (
            jsonify(
                {
                    "code": 409,
                    "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'",
                }
            ),
            409,
        )

    # 检查是否有世界状态数据
    if not supervisor_world_status_log:
        return jsonify({"code": 404, "message": "暂无世界状态数据"}), 404

    try:
        try:
            target_object_id = int(object_id)
        except ValueError:
            return (
                jsonify(
                    {
                        "code": 400,
                        "message": f"无效的object_id: '{object_id}'，必须是整数",
                    }
                ),
                400,
            )

        # 获取最新的世界状态数据
        latest_log_entry = supervisor_world_status_log[-1]
        nodes_data = latest_log_entry.get("data", [])

        # 查找目标物体
        target_object = None
        for i, node in enumerate(nodes_data, 1):
            if i == target_object_id:
                target_object = node
                break

        if not target_object:
            return (
                jsonify({"code": 404, "message": f"未找到ID为 '{object_id}' 的物体"}),
                404,
            )

        position = target_object.get("position", [0, 0, 0])

        # 确保position是列表且长度为3
        if not isinstance(position, list) or len(position) < 3:
            position = [0, 0, 0]

        # 获取该机器人的目标和状态
        robot_goal_state = robot_goals[object_id]
        status_code = robot_goal_state.get("status", 1)
        object_info = {
            "id": target_object_id,
            "status": status_code,
            "position": {
                "x": f"{safe_float(position[0]):.2f}",
                "y": f"{safe_float(position[1]):.2f}",
                "z": f"{safe_float(position[2]):.2f}",
            },
        }

        return jsonify({"code": 200, "message": "success", "data": object_info}), 200

    except Exception as e:
        return jsonify({"code": 500, "message": f"获取物体信息失败: {str(e)}"}), 500


# ================= 轮臂机器人移动相关路由 =================
@app.route("/api/v1/move/set_goal", methods=["POST"])
def move_set_goal():
    """
    轮臂机器人移动命令下发

    请求参数:
    - id: 场景id，需与当前已加载场景id一致
    - object_id: 要移动的机器人的物品id
    - goal_pos: 目标点的行、列坐标
    - goal_angle: 机器人与目标点的目标偏航角，角度制
    """
    global robot_goals, current_loaded_scene, supervisor_world_status_log, robot_status_dict
    # print("current_loaded_scene:", current_loaded_scene)
    try:
        data = request.get_json()
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "object_id", "goal_pos", "goal_angle"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        scene_id = str(data["id"])
        object_id = int(data["object_id"])
        goal_pos = data["goal_pos"]
        if isinstance(goal_pos, list) and len(goal_pos) == 0:
            goal_pos = []
        goal_angle = float(data["goal_angle"])

        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 验证目标位置格式，允许 goal_pos 为 None
        if goal_pos is not None and (
            not isinstance(goal_pos, list)
            or not (len(goal_pos) == 2 or len(goal_pos) == 0)
        ):
            return (
                jsonify({"code": 400, "message": "目标位置必须是二维坐标数组 [x, y]"}),
                400,
            )

        # 验证目标偏航角范围
        if goal_angle < -180 or goal_angle > 180:
            return jsonify({"code": 400, "message": "目标角度超出范围"}), 400

        # 设置机器人目标状态前，先清理旧状态
        if object_id in robot_goals:
            logger.info(f"[set_robot_goal] 清理机器人(ID:{object_id})的旧目标状态")

        new_goal = {
            "object_id": object_id,
            "goal_pos": goal_pos,
            "goal_angle": goal_angle,
            "status": 2,
            "start_time": time.time(),
            "process": "0/0",
            "total_segments": 0,
            "current_segment": 0,  # 初始时段索引为0
            "progress": 0,  # 初始进度为0%
            "is_reachable": True,  # 初始默认可达，直到BFS算法确定不可达
            "path_planning_complete": False,  # 标记路径规划是否完成
            "path_planning_logged": False,  # 重置路径规划日志标记
        }

        if len(goal_pos) == 0:
            logger.info(
                f"[set_robot_goal] 机器人(ID:{object_id})仅设置旋转目标: 角度 {goal_angle}°"
            )
            new_goal["path_planning_complete"] = True
            new_goal["total_segments"] = 0
            new_goal["goal_pos"] = []
            robot_goals[object_id] = new_goal
            robot_status_dict[object_id] = 2
            return (
                jsonify(
                    {
                        "code": 200,
                        "message": "success",
                        "total_segments": 0,
                        "is_reachable": True,
                    }
                ),
                200,
            )

        robot_goals[object_id] = new_goal
        robot_status_dict[object_id] = 2  # 2=移动中

        logger.info(
            f"[set_robot_goal] 设置机器人(ID:{object_id})目标位置: {goal_pos}, 角度: {goal_angle}°"
        )
        request_response_log.append(
            {
                "event_type": "set_goal",
                "object_id": object_id,
                "goal_pos": goal_pos,
                "goal_angle": goal_angle,
                "status": 2,
                "message": "机器人进入移动中",
            }
        )

        # 等待BFS路径规划结果
        max_wait_time = 10  # 最长等待10秒
        wait_interval = 0.1  # 每0.1秒检查一次
        wait_count = 0

        logger.info(f"[set_robot_goal] 等待机器人(ID:{object_id})路径规划结果...")

        while wait_count < max_wait_time / wait_interval:
            if object_id in robot_goals:
                if robot_goals[object_id].get("path_planning_complete", False):
                    if robot_goals[object_id].get("is_reachable") is False:
                        # 路径规划失败，目标不可达
                        error_message = robot_goals[object_id].get(
                            "error_message", f"无法找到到达目标点 {goal_pos} 的可行路径"
                        )
                        logger.warning(
                            f"[set_robot_goal] 机器人(ID:{object_id})路径规划失败: {error_message}"
                        )
                        robot_goals[object_id]["status"] = 4  # 4=错误
                        robot_status_dict[object_id] = 4
                        request_response_log.append(
                            {
                                "event_type": "set_goal_failed",
                                "object_id": object_id,
                                "goal_pos": goal_pos,
                                "error": error_message,
                                "message": "路径规划失败，目标不可达",
                            }
                        )

                        return (
                            jsonify(
                                {
                                    "code": 400,
                                    "message": "路径规划失败",
                                    "error": error_message,
                                    "is_reachable": False,
                                }
                            ),
                            400,
                        )

                    # 路径规划已完成且成功
                    logger.info(
                        f"[set_robot_goal] 机器人(ID:{object_id})路径规划成功，总段数: {robot_goals[object_id].get('total_segments', 0)}"
                    )
                    return (
                        jsonify(
                            {
                                "code": 200,
                                "message": "success",
                                "total_segments": robot_goals[object_id].get(
                                    "total_segments", 0
                                ),
                                "is_reachable": True,
                            }
                        ),
                        200,
                    )

                # 兼容旧版本：检查is_reachable标志和total_segments数量
                elif robot_goals[object_id].get("is_reachable") is False:
                    # 路径规划失败，目标不可达（旧版本兼容）
                    error_message = robot_goals[object_id].get(
                        "error_message", f"无法找到到达目标点 {goal_pos} 的可行路径"
                    )
                    logger.warning(
                        f"[set_robot_goal] 机器人(ID:{object_id})路径规划失败(旧版本检测): {error_message}"
                    )

                    # 更新状态为错误
                    robot_goals[object_id]["status"] = 4  # 4=错误
                    robot_status_dict[object_id] = 4

                    request_response_log.append(
                        {
                            "event_type": "set_goal_failed",
                            "object_id": object_id,
                            "goal_pos": goal_pos,
                            "error": error_message,
                            "message": "路径规划失败，目标不可达(旧版本检测)",
                        }
                    )

                    return (
                        jsonify(
                            {
                                "code": 400,
                                "message": "路径规划失败",
                                "error": error_message,
                                "is_reachable": False,
                            }
                        ),
                        400,
                    )

                # 检查是否有段数信息（表示路径规划可能已成功）
                elif robot_goals[object_id].get("total_segments", 0) > 0:
                    # 路径规划可能已成功（旧版本兼容）
                    logger.info(
                        f"[set_robot_goal] 机器人(ID:{object_id})路径规划成功(旧版本检测)，总段数: {robot_goals[object_id].get('total_segments', 0)}"
                    )
                    return (
                        jsonify(
                            {
                                "code": 200,
                                "message": "success",
                                "total_segments": robot_goals[object_id].get(
                                    "total_segments", 0
                                ),
                                "is_reachable": True,
                            }
                        ),
                        200,
                    )

            # 等待一段时间后再次检查
            time.sleep(wait_interval)
            wait_count += 1

        # 等待超时，返回处理中状态
        logger.warning(f"[set_robot_goal] 等待机器人(ID:{object_id})路径规划结果超时")
        return (
            jsonify(
                {
                    "code": 202,
                    "message": "路径规划处理中，请稍后通过check_state接口查询结果",
                    "is_processing": True,
                }
            ),
            202,
        )
    except Exception as e:
        logger.error(f"设置机器人目标时出错: {str(e)}")
        return jsonify({"code": 400, "message": f"请求处理出错: {str(e)}"}), 400


@app.route("/api/v1/move/check_state", methods=["POST"])
def move_check_state():
    """
    轮臂机器人移动状态获取

    请求参数:
    - id: 场景id，需与当前已加载场景id一致
    - object_id: 要查询的机器人的物品id
    """
    global robot_goals, current_loaded_scene, robot_status_dict

    try:
        data = request.get_json()
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "object_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        scene_id = str(data["id"])
        try:
            object_id = int(data["object_id"])
        except ValueError:
            return jsonify({"code": 400, "message": "物品ID必须为整数"}), 400

        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 获取机器人状态
        status_code = robot_status_dict.get(object_id, 1)  # 默认状态为1(未移动)

        # 初始化返回数据
        response_data = {
            "code": 200,
            "message": "success",
            "status": status_code,
            "process": "0/0",
        }

        # 如果机器人有目标状态记录，添加详细信息
        if object_id in robot_goals:
            goal_state = robot_goals[object_id]
            response_data.update({"process": goal_state.get("process", "0/0")})

            # 如果状态是错误(4)，添加错误信息
            if status_code == 4:
                response_data["error_message"] = "目标点不可达，请选择其他目标点"

        # print(response_data)

        request_response_log.append(
            {
                "event_type": "check_state",
                "object_id": object_id,
                "status": status_code,
                "data": response_data,
                "message": "查询机器人移动状态",
            }
        )

        return jsonify(response_data), 200
    except Exception as e:
        logger.error(f"获取机器人移动状态时出错: {str(e)}")
        return jsonify({"code": 400, "message": f"发生错误: {str(e)}"}), 400


@app.route("/api/v1/move/check_pos", methods=["GET"])
def move_check_pos():
    """
    获取场景的单个物品移动坐标

    请求参数:
    - id: 场景ID
    - object_id: 物品ID
    """
    global supervisor_world_status_log, current_loaded_scene

    try:
        data = request.get_json()
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "object_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        scene_id = str(data["id"])
        try:
            object_id = int(data["object_id"])
        except ValueError:
            return jsonify({"code": 400, "message": "物品ID必须为整数"}), 400

        # # 验证场景ID是否为当前已加载的场景
        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 从最新的世界状态日志中查找指定物体
        if not supervisor_world_status_log:
            return jsonify({"code": 400, "message": "场景数据不可用"}), 400

        latest_log_entry = supervisor_world_status_log[-1]
        target_object = None

        for node in latest_log_entry.get("data", []):
            if node.get("id") == object_id:
                target_object = node
                break

        if not target_object:
            return (
                jsonify({"code": 400, "message": f"未找到ID为{object_id}的物体"}),
                400,
            )

        # 构建响应数据
        position = target_object.get("position", [0, 0, 0])
        rotation = target_object.get("rotation_degrees", {})

        # 确保position是列表且长度为3
        if not isinstance(position, list) or len(position) < 3:
            position = [0, 0, 0]

        response_data = {
            "code": 200,
            "message": "success",
            "data": {
                "id": object_id,
                "position": {
                    "x": f"{safe_float(position[0]):.2f}",
                    "y": f"{safe_float(position[1]):.2f}",
                    "z": f"{safe_float(position[2]):.2f}",
                },
                "orien": {
                    "rx": f"{safe_float(rotation.get('roll', 0)):.2f}",
                    "ry": f"{safe_float(rotation.get('pitch', 0)):.2f}",
                    "rz": f"{safe_float(rotation.get('yaw', 0)):.2f}",
                },
            },
        }

        return jsonify(response_data), 200

    except Exception as e:
        logger.error(f"获取物体位置时出错: {str(e)}")
        return jsonify({"code": 400, "message": f"请求处理出错: {str(e)}"}), 400


@app.route("/api/v1/move/stop", methods=["GET"])
def move_stop():
    """停止机器人移动，清空移动轨迹

    Returns:
        JSON响应，包含操作状态
    """
    global robot_stop_flag

    try:
        data = request.get_json()
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "object_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        scene_id = str(data["id"])
        try:
            object_id = int(data["object_id"])
        except ValueError:
            return jsonify({"code": 400, "message": "物品ID必须为整数"}), 400

        # # 验证场景ID是否为当前已加载的场景
        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 清空所有机器人的目标
        if robot_stop_flag is True:
            return jsonify(
                {
                    "code": 200,
                    "message": f"指令已下发，正在等待其生效",
                    "success": True,
                }
            )
        else:
            robot_stop_flag = True

        # 更新所有机器人的状态为已停止
        robot_status_dict[object_id] = {
            "status": "已停止",
            "message": "机器人已停止移动",
            "is_completed": False,
            "is_stopped": True,
        }

        # 记录操作日志
        logger.info(f"接收到停止命令，已清空机器人的移动目标")
        request_response_log.append(
            {
                "event_type": "robot_stop_command",
                "message": f"已停止机器人的移动",
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
        )
    except Exception as e:
        logger.error(f"获取物体位置时出错: {str(e)}")
        return jsonify({"code": 400, "message": f"请求处理出错: {str(e)}"}), 400

    # 返回响应
    return jsonify(
        {
            "code": 200,
            "message": f"已停止机器人移动",
            "success": True,
        }
    )


# ================= 轮臂机器人抓取相关路由 =================
@app.route("/api/v1/capture/pick_and_place", methods=["POST"])
def capture_set_goal():
    """
    抓取命令接口

    请求参数:
    - id: 场景ID
    - robot_id: 机器人的id
    - cur_pos: 要抓取的目标物体的xyz坐标 ,在目标不可达时返回报错
    - goal_pos:要将物体移动到的目标位置,内容为xyz 坐标
    - goal_angle:要进行抓取的姿态，默认为从上往下抓,[-3.14,0,0]
    - goal_arm:规划左手："left"，右手："right"，双手:"both"
    """
    global capture_robot_goals, current_loaded_scene, supervisor_world_status_log

    try:
        data = request.get_json()
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "robot_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        # scene_id = int(data['id'])
        # robot_id = int(data['robot_id'])
        # goal_arm = str(data['goal_arm'])
        # 获取scene_id，若不存在则默认值为0
        scene_id = int(data.get("id", 1))
        # 获取robot_id，若不存在则默认值为1
        robot_id = int(data.get("robot_id", 1))
        # 获取goal_arm，若不存在则默认值为空字符串
        goal_arm = str(data.get("goal_arm", "both"))

        # left_cur_pos = data['left_cur_pos']
        # left_goal_pos = data['left_goal_pos']
        # left_angle = data['left_angle']
        # right_cur_pos = data['right_cur_pos']
        # right_goal_pos = data['right_goal_pos']
        # right_angle = data['right_angle']

        # 左手臂相关参数，默认值设为合理的空列表或零值
        left_cur_pos = data.get("left_cur_pos", [-0.01749985, 0.29927, -0.21073727])
        left_goal_pos = data.get("left_goal_pos", [-0.01749985, 0.29927, -0.21073727])
        left_angle = data.get("left_angle", [3.14, 0.0, 0.0])

        # 右手臂相关参数，默认值设为合理的空列表或零值
        right_cur_pos = data.get("right_cur_pos", [-0.01749985, -0.29927, -0.21073727])
        right_goal_pos = data.get(
            "right_goal_pos", [-0.01749985, -0.29927, -0.21073727]
        )
        right_angle = data.get("right_angle", [-3.02456926, -0.00675474, 0.09522905])

        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 设置机器人目标状态前，先清理旧状态
        if capture_robot_goals:
            # 使用 robot_id 而非 object_id
            logger.info(f"[set_robot_goal] 清理机器人(ID:{robot_id})的旧目标状态")

        capture_robot_goals["scene_id"] = scene_id
        capture_robot_goals["robot_id"] = robot_id
        capture_robot_goals["goal_arm"] = goal_arm

        capture_robot_goals["left_cur_pos"] = left_cur_pos
        capture_robot_goals["left_goal_pos"] = left_goal_pos
        capture_robot_goals["left_angle"] = left_angle
        capture_robot_goals["right_cur_pos"] = right_cur_pos
        capture_robot_goals["right_goal_pos"] = right_goal_pos
        capture_robot_goals["right_angle"] = right_angle

        # 循环检查字典是否为空（示例：最多等待10次，每次间隔0.1秒）
        max_attempts = 20
        attempt = 0
        while attempt < max_attempts:
            if not capture_robot_goals:  # 字典为空时返回
                return (
                    jsonify(
                        {
                            "code": 200,
                            "message": "目标已清空，操作完成",
                            "cur_pos": [6.1, 6.2, 6.3],
                        }
                    ),
                    200,
                )
            # 模拟等待（实际场景中可能是其他异步操作）
            time.sleep(1)
            attempt += 1

        # 超过最大尝试次数后返回
        return (
            jsonify(
                {
                    "code": 408,
                    "message": "请稍后使用pick_result接口查询",
                }
            ),
            408,
        )

    except Exception as e:
        return (
            jsonify(
                {"code": 400, "message": "The target for capture cannot be reached."}
            ),
            400,
        )


@app.route("/api/v1/capture/pick", methods=["POST"])
def pick_set_goal():
    """
    抓取命令接口

    请求参数:
    - id: 场景ID
    - robot_id: 机器人的id
    - cur_pos: 要抓取的目标物体的xyz坐标 ,在目标不可达时返回报错
    - goal_pos:要将物体移动到的目标位置,内容为xyz 坐标
    - goal_angle:要进行抓取的姿态，默认为从上往下抓,[-3.14,0,0]
    - goal_arm:规划左手："left"，右手："right"，双手:"both"
    """
    global current_left_rpy, current_right_rpy, current_left_pos, current_right_pos, pick_robot_goals, current_loaded_scene, supervisor_world_status_log, result_pick, fail_pick

    try:
        result_pick = False
        data = request.get_json()
        logger.info(f"[set_robot_goal] 接收到抓取命令: {data}")
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "robot_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        # scene_id = int(data['id'])
        # robot_id = int(data['robot_id'])
        # goal_arm = str(data['goal_arm'])
        # 获取scene_id，若不存在则默认值为0
        scene_id = int(data.get("id", 1))
        # 获取robot_id，若不存在则默认值为1
        robot_id = int(data.get("robot_id", 1))
        # 获取goal_arm，若不存在则默认值为空字符串
        goal_arm = str(data.get("goal_arm", "both"))

        # left_cur_pos = data['left_cur_pos']
        # left_goal_pos = data['left_goal_pos']
        # left_angle = data['left_angle']
        # right_cur_pos = data['right_cur_pos']
        # right_goal_pos = data['right_goal_pos']
        # right_angle = data['right_angle']

        # 左手臂相关参数，默认值设为合理的空列表或零值
        left_cur_pos = data.get("left_cur_pos", [0.951, 0.188, 0.790675])
        # left_cur_pos = [0.0, 0.0, 0.0]
        left_angle = data.get("left_angle", [0, 0.0, 0.0])

        # 右手臂相关参数，默认值设为合理的空列表或零值
        right_cur_pos = data.get("right_cur_pos", [0.951, -0.188, 0.790675])
        right_angle = data.get("right_angle", [0, 0, 0])
        logger.info(
            f"{scene_id}, {robot_id}, {goal_arm}, {left_cur_pos}, {left_angle}, {right_cur_pos}, {right_angle}"
        )

        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 设置机器人目标状态前，先清理旧状态
        if pick_robot_goals:
            logger.info(f"[set_robot_goal] 清理机器人(ID:{robot_id})的旧目标状态")

        pick_robot_goals["scene_id"] = scene_id
        pick_robot_goals["robot_id"] = robot_id
        pick_robot_goals["goal_arm"] = goal_arm

        pick_robot_goals["left_cur_pos"] = left_cur_pos
        pick_robot_goals["left_angle"] = left_angle
        pick_robot_goals["right_cur_pos"] = right_cur_pos
        pick_robot_goals["right_angle"] = right_angle

        # 循环检查字典是否为空（示例：最多等待10次，每次间隔0.1秒）
        max_attempts = 30000
        attempt = 0
        while attempt < max_attempts:
            # print("fail_pick",fail_pick)
            if fail_pick:
                fail_pick = False
                return (
                    jsonify(
                        {
                            "code": 400,
                            "message": "The target for capture cannot be reached.",
                        }
                    ),
                    400,
                )
            if not pick_robot_goals:  # 字典为空时返回
                return (
                    jsonify(
                        {
                            "code": 200,
                            "message": "操作完成",
                            "left_real_pos": current_left_pos,
                            "right_real_pos": current_right_pos,
                            "left_real_rpy": current_left_rpy,
                            "right_real_rpy": current_right_rpy,
                        }
                    ),
                    200,
                )
            # 模拟等待（实际场景中可能是其他异步操作）
            time.sleep(1)
            attempt += 1

        # 超过最大尝试次数后返回
        return (
            jsonify(
                {
                    "code": 408,
                    "message": "请稍后使用pick_result接口查询",
                }
            ),
            408,
        )

    except Exception as e:
        return jsonify({"code": 400, "message": "pick fail"}), 400


@app.route("/api/v1/capture/place", methods=["POST"])
def place_set_goal():
    """
    抓取命令接口

    请求参数:
    - id: 场景ID
    - robot_id: 机器人的id
    - cur_pos: 要抓取的目标物体的xyz坐标 ,在目标不可达时返回报错
    - goal_pos:要将物体移动到的目标位置,内容为xyz 坐标
    - goal_angle:要进行抓取的姿态，默认为从上往下抓,[-3.14,0,0]
    - goal_arm:规划左手："left"，右手："right"，双手:"both"
    """
    global current_left_rpy, current_right_rpy, current_left_pos, current_right_pos, place_robot_goals, current_loaded_scene, supervisor_world_status_log, result_place, fail_place

    try:
        result_place = False
        data = request.get_json()
        logger.info(f"[set_robot_goal] 接收到放置命令: {data}")
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "robot_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        # 获取scene_id，若不存在则默认值为0
        scene_id = int(data.get("id", 1))
        # 获取robot_id，若不存在则默认值为1
        robot_id = int(data.get("robot_id", 1))
        # 获取goal_arm，若不存在则默认值为空字符串
        goal_arm = str(data.get("goal_arm", "both"))

        # 左手臂相关参数，默认值设为合理的空列表或零值
        left_goal_pos = data.get("left_goal_pos", [0.951, 0.188, 0.790675])
        left_angle = data.get("left_angle", [0, 0.0, 0.0])

        # 右手臂相关参数，默认值设为合理的空列表或零值
        right_goal_pos = data.get("right_goal_pos", [0.921, -0.188, 0.790675])
        right_angle = data.get("right_angle", [0, 0, 0])

        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 设置机器人目标状态前，先清理旧状态
        if place_robot_goals:
            logger.info(f"[set_robot_goal] 清理机器人(ID:{robot_id})的旧目标状态")

        place_robot_goals["scene_id"] = scene_id
        place_robot_goals["robot_id"] = robot_id
        place_robot_goals["goal_arm"] = goal_arm

        place_robot_goals["left_goal_pos"] = left_goal_pos
        place_robot_goals["left_angle"] = left_angle
        place_robot_goals["right_goal_pos"] = right_goal_pos
        place_robot_goals["right_angle"] = right_angle

        # 循环检查字典是否为空（示例：最多等待10次，每次间隔0.1秒）
        max_attempts = 300
        attempt = 0
        while attempt < max_attempts:
            if fail_place:
                fail_place = False
                return (
                    jsonify(
                        {
                            "code": 400,
                            "message": "The target for capture cannot be reached.",
                        }
                    ),
                    400,
                )
            if not place_robot_goals:  # 字典为空时返回
                return (
                    jsonify(
                        {
                            "code": 200,
                            "message": "操作完成",
                            "left_real_pos": current_left_pos,
                            "right_real_pos": current_right_pos,
                            "left_real_rpy": current_left_rpy,
                            "right_real_rpy": current_right_rpy,
                        }
                    ),
                    200,
                )
            # 模拟等待（实际场景中可能是其他异步操作）
            time.sleep(1)
            attempt += 1
        return (
            jsonify(
                {
                    "code": 408,
                    "message": "请稍后使用place_result接口查询",
                }
            ),
            408,
        )

    except Exception as e:
        return jsonify({"code": 400, "message": f"place fail  {e}"}), 400


@app.route("/api/v1/capture/pick_result", methods=["POST"])
def pick_result():
    global result_pick, result_place

    data = request.get_json()
    logger.info(f"[set_robot_goal] 接收到抓取命令: {data}")
    if not data:
        return jsonify({"code": 400, "message": "参数为空"}), 400

    required_fields = ["id", "robot_id", "object_id"]
    for field in required_fields:
        if field not in data:
            return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

    # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
    #         current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
    #         return jsonify({
    #             "code": 409,
    #             "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
    #         }), 409

    # 获取scene_id，若不存在则默认值为0
    scene_id = int(data["id"])
    # 获取robot_id，若不存在则默认值为1
    robot_id = int(data["robot_id"])
    # 获取goal_arm，若不存在则默认值为空字符串
    object_id = int(data["object_id"])
    # print(scene_id,robot_id,object_id)

    res = result_pick
    if res == True:
        result_pick = False
    return jsonify({"code": 200, "message": res}), 200


@app.route("/api/v1/capture/place_result", methods=["POST"])
def place_result():
    global result_pick, result_place

    data = request.get_json()
    # logger.info(f"[set_robot_goal] 接收到命令: {data}")
    if not data:
        return jsonify({"code": 400, "message": "参数为空"}), 400

    required_fields = ["id", "robot_id", "object_id"]
    for field in required_fields:
        if field not in data:
            return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

    # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
    # current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
    # return jsonify({
    # "code": 409,
    # "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
    # }), 409

    # 获取scene_id，若不存在则默认值为0
    scene_id = int(data["id"])
    # 获取robot_id，若不存在则默认值为1
    robot_id = int(data["robot_id"])
    # 获取goal_arm，若不存在则默认值为空字符串
    object_id = int(data["object_id"])
    # print(scene_id,robot_id,object_id)

    res = result_place
    if res == True:
        result_place = False
    return jsonify({"code": 200, "message": res}), 200


@app.route("/api/v1/capture/get_relative_pos", methods=["POST"])
def get_relative_pos():
    """
    获取物品相对于机器人的相对坐标

    请求参数:
    - id: 场景ID（需与当前加载场景匹配）
    - robot_id: 机器人的ID
    - object_id: 目标物品的ID
    """
    global supervisor_world_status_log, current_loaded_scene

    try:
        # 获取请求参数
        data = request.get_json()
        if not data:
            return (
                jsonify(
                    {"code": 400, "message": "参数为空，请提供场景ID、机器人ID和物品ID"}
                ),
                400,
            )

        # 验证必填参数
        required_fields = ["id", "robot_id", "object_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        # 解析参数
        scene_id = str(data["id"])
        try:
            robot_id = int(data["robot_id"])
            object_id = int(data["object_id"])
        except ValueError:
            return (
                jsonify({"code": 400, "message": "robot_id和object_id必须为整数"}),
                400,
            )

        # 检查是否有世界状态数据
        if not supervisor_world_status_log:
            return (
                jsonify(
                    {"code": 404, "message": "暂无世界状态数据，请确保场景已正确加载"}
                ),
                404,
            )

        # 获取最新的世界状态数据
        latest_log_entry = supervisor_world_status_log[-1]
        nodes_data = latest_log_entry.get("data", [])

        # 查找机器人和目标物品的绝对坐标及机器人朝向
        robot_abs_pos = None  # 机器人绝对坐标 (x, y, z)
        robot_rpy = None  # 机器人朝向 [roll, pitch, yaw]（弧度）
        object_abs_pos = None  # 物品绝对坐标 (x, y, z)

        for node in nodes_data:
            node_id = node.get("id")
            # 匹配机器人ID
            if node_id == robot_id:
                robot_abs_pos = node.get("position", [0, 0, 0])

                # 从rotation_degrees字段获取欧拉角并转换为弧度
                rotation_degrees = node.get("rotation_degrees", {})
                roll_deg = rotation_degrees.get("roll", 0)
                pitch_deg = rotation_degrees.get("pitch", 0)
                yaw_deg = rotation_degrees.get("yaw", 0)

                # 将角度转换为弧度
                robot_rpy = [
                    math.radians(roll_deg),
                    math.radians(pitch_deg),
                    math.radians(yaw_deg),
                ]

                # 确保坐标是长度为3的列表
                if not isinstance(robot_abs_pos, list) or len(robot_abs_pos) < 3:
                    robot_abs_pos = [0, 0, 0]

            # 匹配物品ID
            if node_id == object_id:
                object_abs_pos = node.get("position", [0, 0, 0])
                # 确保坐标是长度为3的列表
                if not isinstance(object_abs_pos, list) or len(object_abs_pos) < 3:
                    object_abs_pos = [0, 0, 0]

        # 检查是否找到机器人和物品
        if robot_abs_pos is None:
            return (
                jsonify({"code": 404, "message": f"未找到ID为 {robot_id} 的机器人"}),
                404,
            )

        if object_abs_pos is None:
            return (
                jsonify({"code": 404, "message": f"未找到ID为 {object_id} 的物品"}),
                404,
            )

        # 提取欧拉角（单位：弧度）
        roll, pitch, yaw = robot_rpy

        # 计算物品相对于机器人的偏移向量（世界坐标系）
        dx = safe_float(object_abs_pos[0]) - safe_float(robot_abs_pos[0])
        dy = safe_float(object_abs_pos[1]) - safe_float(robot_abs_pos[1])
        dz = safe_float(object_abs_pos[2]) - safe_float(robot_abs_pos[2])

        # 计算旋转矩阵（世界坐标系到机器人坐标系的变换）
        # ZYX顺规（yaw-pitch-roll）旋转矩阵
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # 构建旋转矩阵（注意：这里是世界坐标系到机器人坐标系的变换）
        R = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
        )

        # 计算相对坐标（将世界坐标系中的偏移向量转换到机器人坐标系）
        # 注意：这里应该使用旋转矩阵的转置（即逆变换）
        relative_vector = np.dot(R.T, np.array([dx, dy, dz]))
        relative_x = relative_vector[0]
        relative_y = relative_vector[1]
        relative_z = relative_vector[2]

        # 判断是否可以手伸到
        arm_ik = ArmIk(
            model_file="./Webots_PR2_Path_Planning/protos/pr2/pr2.urdf", visualize=False
        )
        q0 = arm_ik.get_init_q()
        if relative_y > 0:  # 大于0用左手
            l_hand_pose = [relative_x, relative_y, relative_z]
            r_hand_pose = [0.921, -0.188, 0.790675]
            suggested_hand = "left"
        else:  # 小于等于0用右手
            l_hand_pose = [0.6673, 0.2719, 1.2414]
            r_hand_pose = [relative_x, relative_y, relative_z]
            suggested_hand = "right"

        sol_q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, [0, 0, 0], [0, 0, 0])
        can_reach = sol_q is not None

        # 整理返回数据
        return (
            jsonify(
                {
                    "code": 200,
                    "message": "success",
                    "data": {
                        # 绝对坐标信息（用于调试和验证）
                        "absolute_coordinates": {
                            "robot": {
                                "x": f"{safe_float(robot_abs_pos[0]):.4f}",
                                "y": f"{safe_float(robot_abs_pos[1]):.4f}",
                                "z": f"{safe_float(robot_abs_pos[2]):.4f}",
                            },
                            "object": {
                                "x": f"{safe_float(object_abs_pos[0]):.4f}",
                                "y": f"{safe_float(object_abs_pos[1]):.4f}",
                                "z": f"{safe_float(object_abs_pos[2]):.4f}",
                            },
                        },
                        # 相对坐标信息（物品相对于机器人）
                        "relative_coordinates": {
                            "x": f"{relative_x:.4f}",  # X轴相对距离（机器人坐标系）
                            "y": f"{relative_y:.4f}",  # Y轴相对距离（机器人坐标系）
                            "z": f"{relative_z:.4f}",  # Z轴相对距离（机器人坐标系）
                        },
                        # 机器人朝向信息（用于调试）
                        "robot_orientation": {
                            "roll_rad": f"{roll:.4f}",  # 绕X轴旋转角度（弧度）
                            "pitch_rad": f"{pitch:.4f}",  # 绕Y轴旋转角度（弧度）
                            "yaw_rad": f"{yaw:.4f}",  # 绕Z轴旋转角度（弧度）
                            "roll_deg": f"{roll_deg:.4f}",  # 绕X轴旋转角度（角度）
                            "pitch_deg": f"{pitch_deg:.4f}",  # 绕Y轴旋转角度（角度）
                            "yaw_deg": f"{yaw_deg:.4f}",  # 绕Z轴旋转角度（角度）
                        },
                        # 新增：逆解结果与手部建议
                        "inverse_kinematics_result": {
                            "can_reach": can_reach,
                            "suggested_hand": suggested_hand,
                        },
                    },
                }
            ),
            200,
        )

    except Exception as e:
        logger.error(f"获取相对坐标时出错: {str(e)}")
        return jsonify({"code": 500, "message": f"服务器错误: {str(e)}"}), 500


@app.route("/api/v1/capture/arm_go_pos", methods=["POST"])
def arm_go_pos():

    global current_left_rpy, current_right_rpy, current_left_pos, current_right_pos, arm_go_pos_robot_goal, current_loaded_scene, supervisor_world_status_log, result_place, fail_place, fail_arm_go_pos

    try:
        result_place = False
        data = request.get_json()
        logger.info(f"[set_robot_goal] 接收到放置命令: {data}")
        if not data:
            return jsonify({"code": 400, "message": "参数为空"}), 400

        required_fields = ["id", "robot_id"]
        for field in required_fields:
            if field not in data:
                return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

        # 获取scene_id，若不存在则默认值为0
        scene_id = int(data.get("id", 1))
        # 获取robot_id，若不存在则默认值为1
        robot_id = int(data.get("robot_id", 1))
        # 获取goal_arm，若不存在则默认值为空字符串
        goal_arm = str(data.get("goal_arm", "both"))

        # 左手臂相关参数，默认值设为合理的空列表或零值
        left_goal_pos = data.get("left_goal_pos", [-0.01749985, 0.29927, -0.21073727])
        left_angle = data.get("left_angle", [3.14, 0.0, 0.0])

        # 右手臂相关参数，默认值设为合理的空列表或零值
        right_goal_pos = data.get(
            "right_goal_pos", [-0.01749985, -0.29927, -0.21073727]
        )
        right_angle = data.get("right_angle", [-3.02456926, -0.00675474, 0.09522905])

        # if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        #     current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        #     return jsonify({
        #         "code": 409,
        #         "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        #     }), 409

        # 设置机器人目标状态前，先清理旧状态
        if arm_go_pos_robot_goal:
            logger.info(f"[set_robot_goal] 清理机器人(ID:{robot_id})的旧目标状态")

        arm_go_pos_robot_goal["scene_id"] = scene_id
        arm_go_pos_robot_goal["robot_id"] = robot_id
        arm_go_pos_robot_goal["goal_arm"] = goal_arm

        arm_go_pos_robot_goal["left_goal_pos"] = left_goal_pos
        arm_go_pos_robot_goal["left_angle"] = left_angle
        arm_go_pos_robot_goal["right_goal_pos"] = right_goal_pos
        arm_go_pos_robot_goal["right_angle"] = right_angle

        # 循环检查字典是否为空（示例：最多等待10次，每次间隔0.1秒）
        max_attempts = 15
        attempt = 0
        while attempt < max_attempts:
            if fail_arm_go_pos:
                fail_arm_go_pos = False
                return (
                    jsonify(
                        {
                            "code": 400,
                            "message": "The target for arm move cannot be reached.",
                        }
                    ),
                    400,
                )
            if not arm_go_pos_robot_goal:  # 字典为空时返回
                return (
                    jsonify(
                        {
                            "code": 200,
                            "message": "操作完成",
                            "left_real_pos": current_left_pos,
                            "right_real_pos": current_right_pos,
                            "left_real_rpy": current_left_rpy,
                            "right_real_rpy": current_right_rpy,
                        }
                    ),
                    200,
                )
            # 模拟等待（实际场景中可能是其他异步操作）
            time.sleep(1)
            attempt += 1
        return (
            jsonify(
                {
                    "code": 408,
                    "message": "请稍后使用place_result接口查询",
                }
            ),
            408,
        )

    except Exception as e:
        return jsonify({"code": 400, "message": f"arm move fail  {e}"}), 400


@app.route("/api/v1/capture/get_reachable_objects", methods=["POST"])
def get_reachable_objects():
    global supervisor_world_status_log
    data = request.get_json()
    if not data:
        return jsonify({"code": 400, "message": "参数为空"}), 400

    required_fields = ["id", "robot_id"]
    for field in required_fields:
        if field not in data:
            return jsonify({"code": 400, "message": f"缺少参数: {field}"}), 400

    scene_id = str(data["id"])
    robot_id = str(data["robot_id"])

    if not supervisor_world_status_log:
        return jsonify({"code": 404, "message": "暂无世界状态数据"}), 404

    # 获取最新世界状态
    latest_log = supervisor_world_status_log[-1]
    nodes_data = latest_log.get("data", [])

    robot_abs_pos, robot_rpy, robot_info = None, None, None

    # 提取机器人位置与姿态
    for node in nodes_data:
        if str(node.get("id")) == robot_id:
            robot_abs_pos = node.get("position", [0, 0, 0])
            rot_deg = node.get("rotation_degrees", {})
            robot_rpy = [
                math.radians(rot_deg.get("roll", 0)),
                math.radians(rot_deg.get("pitch", 0)),
                math.radians(rot_deg.get("yaw", 0)),
            ]
            robot_info = {
                "position": {
                    "x": safe_float(robot_abs_pos[0]),
                    "y": safe_float(robot_abs_pos[1]),
                    "z": safe_float(robot_abs_pos[2]),
                },
                "orientation": {
                    "roll": safe_float(rot_deg.get("roll", 0)),
                    "pitch": safe_float(rot_deg.get("pitch", 0)),
                    "yaw": safe_float(rot_deg.get("yaw", 0)),
                },
            }
            break

    if robot_abs_pos is None or robot_rpy is None:
        return jsonify({"code": 404, "message": f"未找到机器人ID {robot_id}"}), 404

    # 初始化 IK 求解器
    arm_ik = ArmIk(
        model_file="./Webots_PR2_Path_Planning/protos/pr2/pr2.urdf", visualize=False
    )
    q0 = arm_ik.get_init_q()

    # 机器人姿态旋转矩阵
    roll, pitch, yaw = robot_rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )

    reachable_objects = []

    # 遍历所有物品并过滤
    for node in nodes_data:
        # 跳过机器人和非抓取物体
        if str(node.get("id")) == robot_id or node.get("obj_type") == "1":
            continue

        abilities = node.get("ability", [])
        if not abilities or not any("grab" in a for a in abilities):
            continue

        obj_id = node.get("id")
        obj_pos = node.get("position", [0, 0, 0])
        obj_rot = node.get("rotation_degrees", {})

        # 计算相对位置
        dx = safe_float(obj_pos[0]) - safe_float(robot_abs_pos[0])
        dy = safe_float(obj_pos[1]) - safe_float(robot_abs_pos[1])
        dz = safe_float(obj_pos[2]) - safe_float(robot_abs_pos[2])
        relative = np.dot(R.T, np.array([dx, dy, dz]))
        rel_x, rel_y, rel_z = relative

        # 物体姿态约束
        # obj_yaw = math.radians(obj_rot.get("yaw", 0))
        print("rel_x,rel_y,rel_z", rel_x, rel_y, rel_z)

        # 左手 IK（右手固定）
        sol_left = arm_ik.computeIK(
            q0,
            [rel_x, rel_y, rel_z],
            [0.921, -0.188, 0.790675],
            [0, 0, 0],
            [0, 0, 0],
        )

        # 右手 IK（左手固定）
        sol_right = arm_ik.computeIK(
            q0,
            [0.921, 0.188, 0.790675],
            [rel_x, rel_y, rel_z],
            [0, 0, 0],
            [0, 0, 0],
        )

        can_reach = (sol_left is not None) or (sol_right is not None)

        # 只返回可达的物体
        if can_reach:
            reachable_objects.append(
                {
                    "id": obj_id,
                    "name": node.get("name", "未知"),
                    "relative_position": {"x": rel_x, "y": rel_y, "z": rel_z},
                    "absolute_position": {
                        "x": safe_float(obj_pos[0]),
                        "y": safe_float(obj_pos[1]),
                        "z": safe_float(obj_pos[2]),
                    },
                    "orientation": {
                        "roll": safe_float(obj_rot.get("roll", 0)),
                        "pitch": safe_float(obj_rot.get("pitch", 0)),
                        "yaw": safe_float(obj_rot.get("yaw", 0)),
                    },
                }
            )

    return (
        jsonify(
            {
                "code": 200,
                "message": "success",
                "robot": robot_info,
                "reachable_objects": reachable_objects,
            }
        ),
        200,
    )


# ================= 定期打印状态线程 =================
def print_status():
    """定期打印服务器状态到终端以保持连接活跃"""
    PRINT_INTERVAL = 50

    while True:
        try:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            scene_name = (
                current_loaded_scene.get("name", "None")
                if current_loaded_scene
                else "None"
            )

            print(f"\n[{current_time}] 服务器状态:")
            print(f"├─ 当前场景: {scene_name}")
            print(
                f"├─ Webots进程: {'运行中' if webots_process and webots_process.poll() is None else '未运行'}"
            )
            print(f"└─ 服务器运行正常")

            time.sleep(PRINT_INTERVAL)
        except Exception as e:
            logger.error(f"状态打印异常: {e}")
            time.sleep(PRINT_INTERVAL)


# ================= 启动服务器 =================
if __name__ == "__main__":
    try:
        print("=" * 80)
        logger.info("🚀 Webots控制服务器启动中...")
        print("📍 服务器地址: http://localhost:5000")
        print("📖 API文档: http://localhost:5000")
        print("🎮 Webots进程控制、机器人状态管理、相机图像服务已整合")
        print("⏰ 状态打印间隔: 50秒")
        print("🛑 按 CTRL+C 退出")
        print("=" * 80)

        # 加载场景配置文件
        logger.info("🔍 正在加载场景配置...")
        load_scene_configs()

        # 清理临时wbt文件
        logger.info("🧹 正在清理临时wbt文件...")
        cleanup_temp_wbt_files()

        # 更新场景配置，添加label_point信息
        logger.info("正在扫描场景中的标签点...")
        SCENE_CONFIGS = wbt_parser.update_scene_configs_with_label_points(SCENE_CONFIGS)
        logger.info("场景配置更新完成")

        # 启动状态打印线程
        status_thread = threading.Thread(target=print_status, daemon=True)
        status_thread.start()

        logger.info("Flask服务器启动")
        app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)

    except KeyboardInterrupt:
        logger.info("接收到停止信号，正在清理...")

    finally:
        # 清理Webots进程和场景信息
        kill_all_webots_processes()
        current_loaded_scene = None

        # 保存所有日志
        save_main_log_on_exit()
        save_supervisor_log_on_exit()

        logger.info("整合Flask服务器停止")
        logger.info("服务器已安全停止")
