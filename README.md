# Flask 服务器 API 接口

Flask 服务器现在作为 HTTP RESTful API 服务器，支持PR2机器人导航和机械臂高级控制。

---

## 1. 客户端向服务器发送指令 (POST 请求)

客户端通过 POST 请求向服务器发送机器人指令。所有请求体都应为 JSON 格式，且顶层为一个 JSON 对象的列表。

### 1.1 发送导航目标 (`POST /set_goal`)

- **描述：** Web 客户端向服务器发送一个或多个机器人导航目标位置。支持浮点数坐标。

- **请求体 (JSON - 列表):**

  ```
  [
      {
          "type": "navigation",
          "goal": [float, float] // 例如: [7.5, 7.5], 支持浮点数
      },
      {
          "type": "navigation",
          "goal": [1.2, 3.4]
      }
  ]
  ```

- **服务器处理：** 服务器会将列表中的每个导航目标添加到内部命令队列中。

- **响应 (JSON - 200 OK):**

  ```
  [
      {
          "status": "success",
          "message": "Goal(s) added to command queue.",
          "goals_received": [
              [7.5, 7.5],
              [1.2, 3.4]
          ]
      }
  ]
  ```

- **错误响应 (JSON - 400 Bad Request):**

  ```
  [
      {
          "status": "error",
          "message": "Invalid request format. Expected a list of navigation commands.",
          "details": "..."
      }
  ]
  ```

### 1.2 发送机械臂控制指令 (`POST /set_arm_command`)

- **描述：** Web 客户端向服务器发送一个或双机械臂控制指令。支持关节角度控制和末端姿态控制，并可指定抓取物品。

- **请求体 (JSON - 列表):**

  ```
  [
      {
          "type": "joint_control",
          "angles": {
              "r_shoulder_pan_joint": float,      // 关节名称及其目标角度（弧度）
              "r_shoulder_lift_joint": float,
              "r_upper_arm_roll_joint": float,
              "r_elbow_flex_joint": float,
              "r_forearm_roll_joint": float,
              "r_wrist_flex_joint": float,
              "r_wrist_roll_joint": float,
              "l_shoulder_pan_joint": float,
              "l_shoulder_lift_joint": float,
              "l_upper_arm_roll_joint": float,
              "l_elbow_flex_joint": float,
              "l_forearm_roll_joint": float,
              "l_wrist_flex_joint": float,
              "l_wrist_roll_joint": float,
              "torso_lift_joint": float           // 躯干高度
          },
          "gripper_command": "open" | "close" | "none" // 抓手指令 (可选)
      },
      {
          "type": "pose_control",
          "arm": "right" | "left",                  // 控制哪个机械臂
          "target_pose": {                          // 目标末端姿态 (相对于机器人基座)
              "x": float,
              "y": float,
              "z": float,
              "roll": float,                        // 姿态的欧拉角 (弧度) 或四元数
              "pitch": float,
              "yaw": float
              // 或者 "orientation": {"x": float, "y": float, "z": float, "w": float} (四元数)
          },
          "gripper_command": "open" | "close" | "release" | "none", // 抓手指令 (可选)
          "item_id": "string",                      // 待抓取/放置物品的 Webots 节点名称或 ID (可选)
          "item_coordinates": [float, float, float] // 待抓取/放置物品的 3D 坐标 (可选)
      }
  ]
  ```

- **服务器处理：** 服务器会将列表中的每个机械臂指令添加到内部命令队列中。

- **响应 (JSON - 200 OK):**

  ```
  [
      {
          "status": "success",
          "message": "Arm command(s) added to command queue.",
          "commands_received": [
              {
                  "type": "joint_control",
                  "angles": {...}
              },
              {
                  "type": "pose_control",
                  "arm": "right",
                  "target_pose": {...}
              }
          ]
      }
  ]
  ```

- **错误响应 (JSON - 400 Bad Request):**

  ```
  [
      {
          "status": "error",
          "message": "Invalid request format. Expected a list of arm commands.",
          "details": "..."
      }
  ]
  ```

---



## 2. Webots 控制器获取指令 (`GET /robot_command`)

- **描述：** Webots 控制器向服务器轮询待执行的命令（导航或机械臂）。服务器会检查内部命令队列。**注意：** 服务器会按顺序从队列中取出命令。

- **请求参数：** 无。

- **服务器处理：**

  1.  从内部命令队列中取出下一个待执行的命令。
  2.  如果队列为空，则返回一个空列表。

- **响应 (JSON - 200 OK):**

  - 返回一个包含单个命令的列表，或者一个空列表 `[]`。

  - **导航指令示例：**

    ```
    [
        {
            "type": "navigation",
            "goal": [float, float]
        }
    ]
    ```

  - **关节角度控制指令示例：**

    ```
    [
        {
            "type": "joint_control",
            "angles": {
                "l_shoulder_pan_joint": float,
                // ... 其他关节
            },
            "gripper_command": "open" | "close" | "none"
        }
    ]
    ```

  - **末端姿态控制指令示例：**

    ```
    [
        {
            "type": "pose_control",
            "arm": "right" | "left",
            "target_pose": {
                "x": float, "y": float, "z": float,
                "roll": float, "pitch": float, "yaw": float
            },
            "gripper_command": "open" | "close" | "release" | "none",
            "item_id": "string",
            "item_coordinates": [float, float, float]
        }
    ]
    ```

  - **无指令时：**

    ```
    []
    ```

---



## 3. Webots 控制器发送机器人状态 (`POST /robot_status`)

- **描述：** Webots 控制器向服务器发送机器人（包括机械臂）的当前复合状态。

- **请求体 (JSON - 列表):**

  ```
  [
      {
          "timestamp": float,                          // 时间戳
          "base_position": [float, float],             // 机器人底盘在地图上的 (X, Y) 坐标 (浮点数)
          "base_orientation_rad": float,               // 机器人底盘的当前朝向 (弧度)
          "status_message": "string",                  // 当前机器人状态描述 (例如: "Moving", "Idle", "Goal reached", "Path blocked")
          "arm_state": {
              "left_arm_joints": {
                  "l_shoulder_pan_joint": float,       // 左臂各关节当前角度 (弧度)
                  "l_shoulder_lift_joint": float,
                  "l_upper_arm_roll_joint": float,
                  "l_elbow_flex_joint": float,
                  "l_forearm_roll_joint": float,
                  "l_wrist_flex_joint": float,
                  "l_wrist_roll_joint": float
              },
              "right_arm_joints": {
                  "r_shoulder_pan_joint": float,       // 右臂各关节当前角度 (弧度)
                  "r_shoulder_lift_joint": float,
                  "r_upper_arm_roll_joint": float,
                  "r_elbow_flex_joint": float,
                  "r_forearm_roll_joint": float,
                  "r_wrist_flex_joint": float,
                  "r_wrist_roll_joint": float
              },
              "torso_height": float,                   // 躯干当前高度
              "left_gripper_position": float,          // 左抓手当前位置 (开合程度)
              "right_gripper_position": float,         // 右抓手当前位置
              "is_holding_item": boolean,              // 是否夹取到了物品
              "held_item_id": "string" | null          // 如果夹取了物品，提供物品的 ID
          }
      }
  ]
  ```

- **服务器处理：** 服务器会更新全局的机器人状态变量，并可能将状态记录到日志中。

- **响应 (JSON - 200 OK):**

  ```
  [
      {
          "status": "success",
          "message": "Robot status updated."
      }
  ]
  ```

- **错误响应 (JSON - 400 Bad Request):**

  ```
  [
      {
          "status": "error",
          "message": "Invalid status data format.",
          "details": "..."
      }
  ]
  ```

---



## 4. Web 客户端获取机器人状态 (`GET /robot_status`)

- **描述：** Web 客户端从服务器获取机器人（包括机械臂）的当前复合状态。

- **请求参数：** 无。

- **服务器处理：** 服务器返回当前存储的机器人状态。

- **响应 (JSON - 200 OK):**

  - 返回一个包含单个状态对象的列表。格式与 `POST /robot_status` 的请求体相同。

  ```
  [
      {
          "timestamp": float,
          "base_position": [float, float],
          "base_orientation_rad": float,
          "status_message": "string",
          "arm_state": {
              "left_arm_joints": {...},
              "right_arm_joints": {...},
              "torso_height": float,
              "left_gripper_position": float,
              "right_gripper_position": float,
              "is_holding_item": boolean,
              "held_item_id": "string" | null
          }
      }
  ]
  ```

- **错误响应 (JSON - 500 Internal Server Error):**

  ```
  [
      {
          "status": "error",
          "message": "Failed to retrieve robot status.",
          "details": "..."
      }
  ]
  ```

---



## 5. Supervisor 控制器发送世界物体状态 (`POST /world_status`)

- **描述：** Webots 中的 Supervisor 控制器定期（例如每秒一次）向 Flask 服务器发送仿真世界中指定物体（机器人和障碍物）的实时状态数据。这些数据用于监控和日志记录。

- **请求体 (JSON):**

  ```
  {
      "timestamp": float, // Unix 时间戳，表示数据发送时间
      "nodes": [          // 监控到的节点列表
          {
              "name": "string",       // 节点的 DEF 名称 (例如 "PR2_ROBOT", "WOODEN_BOX_1")
              "position": [float, float, float], // 节点在世界坐标系中的 [X, Y, Z] 位置 (米)
              "rotation_degrees": {   // 节点的欧拉角旋转 (度)
                  "roll": float,      // 绕 X 轴的翻滚角
                  "pitch": float,     // 绕 Y 轴的俯仰角
                  "yaw": float        // 绕 Z 轴的偏航角
              },
              "is_moving": boolean    // 节点是否正在移动或旋转 (true/false)
          },
          // ... 更多监控到的节点对象 ...
      ]
  }
  ```

- **请求示例：**

  ```
  {
      "timestamp": 1749458633.1826954,
      "nodes": [
          {
              "name": "PR2_ROBOT",
              "position": [
                  -2.499,
                  -3.5,
                  -0.028
              ],
              "rotation_degrees": {
                  "roll": 0.01,
                  "pitch": 1.87,
                  "yaw": -0.23
              },
              "is_moving": true
          },
          {
              "name": "WOODEN_BOX_1",
              "position": [
                  -2.0,
                  0.0,
                  0.5
              ],
              "rotation_degrees": {
                  "roll": 0.0,
                  "pitch": -0.0,
                  "yaw": 0.0
              },
              "is_moving": false
          }
      ]
  }
  ```

- **服务器处理：** 服务器接收到数据后，会将其记录到 `supervisor_world_status_log.json` 文件中。在保存到文件时，`timestamp` 会被转换为可读的日期时间格式。

- **响应 (JSON - 200 OK):**

  ```
  {
      "message": "World status updated"
  }
  ```



## 6. Flask 服务器 (`flask_server.py`) 内部实现调整提示

为了支持上述 API 接口，`flask_server.py` 需要进行以下关键调整：

1.  **全局命令队列：**
    * 使用 `collections.deque` 来存储待执行的命令。
    * `POST /set_goal` 和 `POST /set_arm_command` 将命令 (`dict` 类型) 添加到队列中。
    * `GET /robot_command` 从队列左侧弹出命令。
    * `robot_goal` 和 `robot_arm_command` 可以被队列取代或者用于快速处理特定命令。统一使用队列。

2.  **复合状态存储：**
    * `robot_status` 全局变量需要扩展以存储所有机械臂关节角度、躯干高度、抓手位置以及 `is_holding_item` 和 `held_item_id`。

3.  **JSON 列表处理：**
    * 所有接收 `POST` 请求的路由，在解析 `request.get_json()` 后，需要确保处理的是一个列表。如果不是列表，则返回 `400 Bad Request`。
    * 所有 `GET` 请求的响应，即使只返回一个对象，也将其包装在列表中。

4.  **Webots 控制器 (`BFS_exercise_1.py`) 调整提示：**
    * `POST /robot_status`：修改 `send_robot_status` 函数，使其收集并发送所有机器人（包括机械臂）的复合状态数据。需要从 `MyCustomRobot` 实例中获取这些数据。
    * `GET /robot_command`：修改控制器轮询命令的逻辑以处理可能返回的 `joint_control` 或 `pose_control` 类型指令，并根据 `item_id` 和 `item_coordinates` 调用相应 IK 算法和抓取功能。如果未返回指令（即空列表），则继续等待。

**IK 算法和物品抓取注意事项：**

* **Webots API：** Webots 提供了 `wb_robot_get_device` 和 `wb_motor_set_position` 等函数来控制关节。对于抓取，需要利用碰撞检测（例如通过 `TouchSensor` 或者 `wb_robot_get_contact_points`）来判断是否成功抓取物品，并可能需要使用 `wb_node_set_joint_parameter` 或直接在 Webots 世界文件中修改物品的父子节点关系来模拟抓取。
* **IK 库：** Python 中有像 `PyKDL`、`OpenRave` (更复杂)、`Modern Robotics` 库可用于逆运动学计算。需要在 Webots 控制器中集成这些库，或者自己实现简单 IK 算法（更新中，尚未做到）。
* **物品信息：** 客户端发送的 `item_id` 和 `item_coordinates` 应该足以让 Webots 控制器识别目标物品并计算抓取路径。







## 7. 工程文件和部署说明

此部分将提供flask与pr2机器人bfs算法路径规划系统的工程文件结构以及部署运行的详细步骤，以便于复现和调试。

#### 7.1 工程文件结构

flask服务器项目根目录为 `webots_server_http/`，文件结构如下：

```
webots_server_http/
├── logs/
│   └── flask_server_activity_log.json    # 打印flask服务器活动日志
│   └── supervisor_world_status_log.json  # 打印supervisor世界物体状态日志
├── flask_server.py                       # Flask后端server代码
├── templates/
│   └── index.html                        # web-client端文件
├── requirements.txt                      # Python依赖文件
└── README.md                             # 项目说明文档
```

`flask_server.py`: 负责处理所有HTTP请求，维护机器人状态和命令队列。

`templates/index.html`: 一个简单网页界面，用于用户设置目标和查看机器人状态（如果需要提供）。

`requirements.txt`: 包含所有Python依赖的列表。



仿真控制脚本根目录为`Webots_PR2_Path_Planning/`，文件结构如下：

```
Webots_PR2_Path_Planning/
├── controllers/
│   └── supervisor_monitor/
│       └── supervisor_monitor.py # Webots场景物体监视代码，在Webots仿真环境中监视场景物体的控制器，负责实时反馈世界状态
│   └── BFS_exercise_1
│       └── BFS_exercise_1.py     # Webots机器人控制器代码，在Webots仿真环境中运行的机器人控制器，负责执行命令并反馈状态
│   └── utilities.py              # PR2机器人底盘移动与上肢机械臂初始化功能代码
├── protos/
│   └── Pr2.proto                 # 机器人节点文件
│   └── meshes/                   # 机器人组件
│   └── textures/                 # 机器人外观贴图
│   └── icons/                    # 机器人图标
├── world/
│   └── lab6_path_planning.wbt    # 仿真场景的世界组成文件（包括机器人节点）
```



#### 7.2 部署步骤

请按照以下步骤部署和运行系统：

1. **环境准备:**

   - 安装Python 3.10+。
   - 安装Webots仿真软件。
   - 建议使用虚拟环境管理Python依赖。

2. **克隆或下载项目:** 将上述工程文件结构下载到本地机子上。

3. **安装Python依赖:** 打开终端，导航到 `webots_server_http/` 目录，然后运行以下命令安装所需的Python库：

   ```bash
   pip install -r requirements.txt
   ```
   
   `requirements.txt` 示例内容：

   ```
   Flask
   requests
   ```
   
4. **运行Flask服务器:** 在终端运行flask服务器：

   ```bash
   python flask_server.py
   ```
   
   服务器默认将在端口上运行：http://127.0.0.1:5000

5. **配置和运行Webots仿真:**

   - 打开Webots软件：导航到`..\Webots\msys64\mingw64\bin`路径下终端输入`./webots --stream`。
   - 加载机器人仿真场景：包含PR2机器人的场景，此时web端的W3D页面地址为http://localhost:1234/index.html
   - 确保机器人控制器（`BFS_exercise_1.py`）已经配置为该机器人控制器。
   - 在 `BFS_exercise_1.py` 中，确保Flask服务器地址和端口设置正确（例如，如果Flask运行在默认端口，则API请求将发送到 `http://127.0.0.1:5000`）。
   - 启动Webots仿真。控制器将开始与Flask服务器通信。

6. **使用Web客户端或API工具进行交互:**

   - **通过Web客户端 (可选):** 在浏览器中打开 `templates/index.html`，通过界面设置机器人目标。
   - **通过API工具 (推荐):** 使用Postman、curl或其他HTTP客户端工具，按照2.1和2.2节的接口说明，向Flask服务器发送请求以控制机器人或获取其状态。



## 8.问题解决：

#### 8.1 对象 ID 获取：当射线击中某个对象时，如何获取该对象的唯一 ID？

在 `webots.js` 中，`webots.currentView.x3dScene.pick(x, y)` 函数是负责执行射线投射并获取被点击对象 ID 的关键部分。

1. **`webots.currentView.x3dScene.pick(x, y)`**: 当鼠标点击事件发生时，会调用此函数，并传入点击的屏幕坐标 `x` 和 `y`。此函数内部会执行射线投射。
2. **射线投射 (Ray Casting)**: 在 `W3dScene.js` 文件中，`W3dScene.prototype.pick` 函数负责处理 3D 拾取逻辑。它将 2D 屏幕坐标转换为 3D 世界中的射线，并检测这条射线与场景中几何体的交点。
3. **获取 Node ID**: `W3dScene.prototype.pick` 函数会遍历场景图，并使用 X3D 或 Webots 内部的数据结构来确定射线击中了哪个 X3D 节点。一旦确定了被击中的节点，它的唯一 ID（通常是 Webots 为每个场景节点分配的唯一标识符）就会被返回。



#### 8.2 与 Webots 仿真服务器的 WebSocket 通信：将选定对象的 ID 发送回 Webots 仿真服务器，以便服务器可以相应地处理（例如，显示白色边框）。这部分在哪里？

在 `webots.js` 中，`webots.currentView.onpick` 函数是处理对象拾取后将 ID 发送回服务器的核心部分。

1. **`webots.currentView.onpick(value)`**: 当 `webots.currentView.x3dScene.pick(x, y)` 函数返回一个有效的对象 ID 后，这个 ID 会作为 `value` 参数传递给 `onpick` 函数。
2. **发送 WebSocket 消息**: `onpick` 函数会检查 `value` 是否有效（即是否成功拾取到一个对象）。如果有效，它会构建一个 WebSocket 消息，通常包含所选对象的 ID。
3. **`webots.currentView.socket.send(message)`**: 这行代码会将包含对象 ID 的消息通过 WebSocket 连接发送到 Webots 仿真服务器。
4. **服务器处理**: Webots 仿真服务器接收到这条消息后，会根据其中的对象 ID 来识别被选中的对象，并执行相应的操作，例如，在 3D 视图中渲染一个白色的选择框来高亮显示该对象。

简而言之，`webots.js` 负责协调用户界面事件（如鼠标点击）和与 3D 场景的交互，而 `W3dScene.js` 提供了底层的 3D 场景渲染和拾取功能。通过 WebSocket 连接，客户端将用户的选择信息发送回 Webots 仿真服务器，实现双向通信和场景的动态更新。



#### 8.3 要测试获取的 ID 是否为鼠标点击的物品，需要通过以下步骤进行验证和调试：

1. **在 `webots.js` 中添加调试输出 (Console Logging)**：

   - 找到 `webots.currentView.onpick(value)` 函数。

     ```javascript
     webots.currentView.onpick = function(value) {
       console.log("Picked object ID:", value); // 添加这行
       if (typeof value !== 'undefined') {
         // ... (rest of the original code)
       }
     };
     ```

     这将允许在浏览器的开发者工具中看到每次点击时 Webots 客户端捕获到的 ID。

2. **获取 Webots 仿真中物品的 ID**：

   - 在 Webots 桌面应用中查看 ID：

     - 打开Webots 仿真。
     - 在场景树（Scene Tree）中选中想要测试的物品。
     - 通常在“节点参数”（Node Parameters）或“高级”（Advanced）视图中可以找到该节点的唯一 ID 或名称。这个 ID 是 Webots 内部用来标识该节点的。

   - 通过 Webots 控制器获取 ID：

     - 如果正在编写 Webots 控制器（例如Python 或 C++），可以使用 

       ```
       getFromDef()
       ```

        或 

       ```
       getFromId()
       ```

        函数来获取特定节点的引用，然后可以打印其 ID。例如：

       ```python
       # Python controller example
       robot = self.getSelf()
       my_object = robot.getFromDef("MY_OBJECT_DEF_NAME") # Or getFromId(some_known_id)
       if my_object:
           print("Webots object ID:", my_object.getId())
       ```

3. **对比 ID**：

   - 在 Web 浏览器中运行Webots stream流式传输。
   - 打开浏览器的开发者工具。
   - 切换到“控制台”（Console）选项卡。
   - 用鼠标点击 Webots 场景中的一个物品。
   - 观察控制台输出的 `Picked object ID: [your_id]`。
   - 将这个 `[your_id]` 与在步骤 2 中从 Webots 桌面应用或通过控制器获取到的物品的实际 ID 进行对比。如果它们匹配说明拾取功能正常工作。

通过以上步骤可以有效地测试并验证 `webots.js` 中获取到的 ID 是否确实对应鼠标点击的物品。
