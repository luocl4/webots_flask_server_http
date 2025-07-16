from controller import Supervisor
import time
import requests

WEB_SERVER_URL = "http://127.0.0.1:5000/sl_server"

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
            command_data = response.json()
            objects = command_data.get("objects", [])
            state_name = command_data.get("state_name", "")
            print(f"对{len(objects)}个对象执行操作，状态名: {state_name}")
            for obj in objects:
                node = supervisor.getFromDef(obj)
                if node:
                    if command_data.get("command") == "save":
                        node.saveState(state_name)
                    elif command_data.get("command") == "load":
                        node.loadState(state_name)
                    else:
                        print("No valid command received.")
        else:
            print(f"HTTP请求失败，状态码: {response.status_code}")
    except Exception as e:
        print(f"主循环发生错误: {e}")
        time.sleep(0.4)