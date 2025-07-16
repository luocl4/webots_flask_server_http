from controller import Supervisor
import time
import requests

WEB_SERVER_URL = "http://127.0.0.1:5000/get_sl_status"

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

counter_1 = 0
HTTP_REQUEST_INTERVAL = 30
MAIN_LOOP_SLEEP = 0.1

while supervisor.step(timestep) != -1:
    try:
        time.sleep(MAIN_LOOP_SLEEP)
        counter_1 += 1
        if counter_1 < HTTP_REQUEST_INTERVAL:
            continue
        counter_1 = 0
        
        try:
            response = requests.get(WEB_SERVER_URL, timeout=2)  # 添加超时设置
        except requests.exceptions.Timeout:
            print("HTTP请求超时，跳过本次请求")
            continue
        except requests.exceptions.ConnectionError:
            print("连接错误，服务器可能不可用")
            time.sleep(0.4)
            continue
        except Exception as e:
            print(f"HTTP请求发生未知错误: {e}")
            continue
            
        if response.status_code == 200:
            try:
                command_data = response.json()
                
                # 检查返回的数据类型
                if isinstance(command_data, list):
                    # 如果是列表，处理列表中的每个对象
                    if len(command_data) > 0:
                        print(f"收到{len(command_data)}个对象的状态信息")
                        for item in command_data:
                            if isinstance(item, dict):
                                object_id = item.get("object_id", "")
                                state_name = item.get("state_name", "")
                                print(f"对象ID: {object_id}, 状态: {state_name}")
                                
                                # 根据object_id获取节点并操作
                                node = supervisor.getFromDef(object_id)
                                if node:
                                    # 这里需要根据实际的command来决定是save还是load
                                    # 由于当前接口返回的数据中没有command字段，
                                    # 您可能需要修改服务器端代码或者在这里添加默认逻辑
                                    print(f"找到节点: {object_id}")
                                    # 默认执行save操作，您可以根据需要修改
                                    # node.saveState(state_name)
                                    print(f"准备保存对象 {object_id} 的状态: {state_name}")
                                else:
                                    print(f"未找到节点: {object_id}")
                            else:
                                print(f"无效的数据项: {item}")
                    else:
                        print("收到空的状态列表")
                        
                elif isinstance(command_data, dict):
                    # 如果是字典，使用原有的处理逻辑
                    objects = command_data.get("objects", [])
                    state_name = command_data.get("state_name", "")
                    command = command_data.get("command", "")
                    print(f"对{len(objects)}个对象执行{command}操作，状态名: {state_name}")
                    for obj in objects:
                        node = supervisor.getFromDef(obj)
                        if node:
                            if command == "save":
                                node.saveState(state_name)
                                print(f"已保存对象 {obj} 的状态: {state_name}")
                            elif command == "load":
                                node.loadState(state_name)
                                print(f"已加载对象 {obj} 的状态: {state_name}")
                            else:
                                print(f"未知命令: {command}")
                        else:
                            print(f"未找到节点: {obj}")
                else:
                    print(f"意外的数据类型: {type(command_data)}")
                    
            except Exception as e:
                print(f"处理响应数据时发生错误: {e}")
                
        elif response.status_code == 404:
            print("服务器返回404：接口不存在或状态未设置")
        else:
            print(f"HTTP请求失败，状态码: {response.status_code}")
            try:
                error_data = response.json()
                print(f"错误信息: {error_data}")
            except:
                print(f"响应内容: {response.text}")
    except Exception as e:
        print(f"主循环发生错误: {e}")
        time.sleep(0.4)