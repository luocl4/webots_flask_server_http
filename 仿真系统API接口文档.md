# 仿真系统API接口  数据字典

## 域名信息

#### 正式域名:  
#### 预发布域名: 
#### 测试域名:  http://clusters.luo980.com:12500
#### dev域名:  http://clusters.luo980.com:12500


## 接口列表
### 

### ide后端和仿真系统交互的场景相关接口
- [查询所有场景信息](#查询所有场景信息)
- [启动加载单个场景](#启动加载单个场景)
- [获取场景的多个物品信息](#获取场景的多个物品信息)
- [获取场景的单个物品信息](#获取场景的单个物品信息)
- [获取场景的单个物品移动坐标](#获取场景的单个物品移动坐标)

### 机器人能力相关接口
- [轮臂机器人移动命令下发](#轮臂机器人移动命令下发)
- [轮臂机器人移动状态获取](#轮臂机器人移动状态获取)
- [获取场景的单个物品移动坐标](#获取场景的单个物品移动坐标)
- [轮臂机器人抓取并放置命令](#轮臂机器人抓取并放置命令)
- [轮臂机器人抓取命令](#轮臂机器人抓取命令)
- [轮臂机器人放置命令](#轮臂机器人放置命令)


## 查询所有场景信息

### 请求地址
`GET /api/v1/scenes`

`请求url格式例子：/api/v1/scenes`

### 请求参数
| 字段名  | 类型     | 必填 | 说明   | 示例值    |
|------|--------|------|------|--------|

### 响应参数
| 字段名            | 类型     | 说明                                | 示例值                                                                                    |
|----------------|--------|-----------------------------------|----------------------------------------------------------------------------------------|
| code           | int    | 状态码                               | 200                                                                                    |
| message        | string | 消息                                | "success"                                                                              |
| data           | array  | 场景列表数据                            | -                                                                                      |
| └─ id          | int    | 场景ID                              | 1                                                                                      |
| └─ name        | string | 场景名称                              | "场景名称"                                                                                 |
| └─ version     | string | 场景版本                              | "v1.2.0"                                                                               |
| └─ description | string | 场景描述                              | "基于ResNet50的图像分类模型"                                                                    |
| └─ size        | string | 场景文件的大小                           | "看看詹小可那边能不能提供场景3d文件大小"                                                                 |
| └─ coordinate  | array  | 场景的四个点坐标(坐标位置数据排序：右上角，左下角，右下角，左上角) | [{"x":"1", "y":"1", "z":"1"},{"x":"1", "y":"1", "z":"1"},{"x":"1", "y":"1", "z":"1"},{"x":"1", "y":"1", "z":"1"}] |
| └─ label_point | array  | 标注点坐标    | [{"p1":{"x":"1", "y":"1", "z":"1"}}]                                                   |

示例
``` json
{
    "code": 200,
    "data": [
        {
            "description": "Kuavo机器人场景",
            "id": 1,
            "name": "Kuavo Robot Scene",
            "scene_id": "kuavo",
            "size": "2KB",
            "version": "1.0.0"
             "coordinate": [{"x":"1", "y":"1", "z":"1"},{"x":"1", "y":"1", "z":"1"},{"x":"1", "y":"1", "z":"1"},{"x":"1", "y":"1", "z":"1"}]
             "label_point": [{"p1":{"x":"1", "y":"1", "z":"1"}},{"p2":{"x":"1", "y":"1", "z":"1"}},{"p3":{"x":"1", "y":"1", "z":"1"}}]
        },
        {
            "description": "路径规划场景",
            "id": 2,
            "name": "Path Planning Scene",
            "scene_id": "path_planning",
            "size": "2KB",
            "version": "2.1.0",
            "coordinate": [],
            "label_point": [],
        }
    ],
    "message": "success"
}
```

## 启动加载单个场景

### 请求地址
`GET /api/v1/scene/load`

`请求url格式例子：/api/v1/scene/load?id=path_planning`

### 请求参数
| 字段名 | 类型 | 必填 | 说明 | 示例值 |
|--------|------|------|------|--------|
| id | int64 | 是 | 场景ID | 1 |

### 响应参数
| 字段名 | 类型 | 说明   | 示例值                    |
|--------|------|------|------------------------|
| code | int | 状态码  | 200                    |
| message | string | 消息   | "success"              |
| data | object | 数据   |                        |
| └─ pid          | int    |    | 1                      |
| └─ scene_id        | string | 场景唯一标识 | "path_planning"                 |
| └─ scene_name     | string | 场景名称 | "v1.2.0"               |
| └─ status | string | 场景描述 | "started"    |

示例
``` json
{
    "code": 200,
    "data": {
        "pid": 4143,
        "scene_id": "path_planning",
        "scene_name": "Path Planning Scene",
        "status": "started"
    },
    "message": "success"
}
```

## 获取场景的多个物品信息

### 请求地址
`GET /api/v1/scene/objects`

`请求url格式例子：/api/v1/scene/objects?id=path_planning`

### 请求参数
| 字段名 | 类型 | 必填 | 说明 | 示例值 |
|--------|------|------|------|--------|
| id | int64 | 是 | 场景ID | 1 |

### 响应参数
| 字段名                              | 类型   | 说明               | 示例值                                   |
|----------------------------------|------|------------------|---------------------------------------|
| code                             | int  | 状态码              | 200                                   |
| message                          | string | 消息               | "success"                             |
| data                             | object | 数据               | null                                  |
| └─ total_count                   | int  | 总数               | 50                                    |
| └─ list                          | array | 数据列表             | -                                     |
| &nbsp;&nbsp;&nbsp;├─ id          | int  | 物品id             | 1                                     |
| &nbsp;&nbsp;&nbsp;├─ name        | string | 物品名称             | "cleaner T20"                         |
| &nbsp;&nbsp;&nbsp;├─ color       | string | 颜色（获取不到就不做了）             | "#4057D4"                             |
| &nbsp;&nbsp;&nbsp;├─ ability_name | array | 能力中文名称集合         | ["抓取","移动","跳动","转动"]                 |
| &nbsp;&nbsp;&nbsp;├─ ability_code | array | 能力英文标识集合         | ["grab","move","jump","turn"]         |
| &nbsp;&nbsp;&nbsp;├─ describe    | string | 描述               | "物品描述"                                |
| &nbsp;&nbsp;&nbsp;├─ rotation    | float  | 旋转角度     | 50.2                           |
| &nbsp;&nbsp;&nbsp;├─ obj_type    | int    | 物品类型     | 1.机器人 2.可交换塑料箱 3.传送带 4.障碍物架子   |
| &nbsp;&nbsp;&nbsp;├─ position    | array | 坐标位置             | ["x":"12.22","y":"12.22","z":"12.22"] |
| &nbsp;&nbsp;&nbsp;├─ size        | array  | 大小（长宽高 ）     | ["length":"1","wide":"2","high":"3"]  |

示例
``` json
{
    "code": 200,
    "data": {
        "list": [
            {
                "ability_code": [
                    "move",
                    "grab",
                    "navigate",
                    "sense"
                ],
                "ability_name": [
                    "移动",
                    "抓取",
                    "导航",
                    "感知"
                ],
                "describe": "PR2双臂服务机器人，具备移动和操作能力",
                "id": 1,
                "name": "PR2 Robot",
                "position": {
                    "x": "-2.50",
                    "y": "-3.50",
                    "z": "-0.03"
                },
                "rotation": 50.2,  
                "obj_type": 1,  
                "size": ["length":"1","wide":"2","high":"3"]
            } 
        ],
        "total_count": 6
    },
    "message": "success"
}
```

## 获取场景的单个物品信息

### 请求地址
`GET /api/v1/scene/object`

`请求url格式例子：/api/v1/scene/object?id=path_planning&object_id=1`

### 请求参数
| 字段名       | 类型 | 必填 | 说明 | 示例值 |
|-----------|------|------|------|--------|
| id        | int64 | 是 | 场景ID | 1 |
| object_id | int64 | 是 | 物品id | 1 |

### 响应参数
| 字段名                              | 类型     | 说明           | 示例值                                   |
|----------------------------------|--------|--------------|---------------------------------------|
| code                             | int    | 状态码          | 200                                   |
| message                          | string | 消息           | "success"                             |
| data                             | object | 数据           | null                                  |
| ├─ id                            | int    | 物品id         | 1                                     |
| ├─ name                          | string | 物品名称         | "cleaner T20"                         |
| ├─ color                         | string | 颜色（获取不到就不做了） | "#4057D4"                             |
| ├─ ability_name | array  | 能力中文名称集合     | ["抓取","移动","跳动","转动"]                 |
| ├─ ability_code | array  | 能力英文标识集合     | ["grab","move","jump","turn"]         |
| ├─ describe    | string | 描述           | "物品描述"                                |
| ├─ rotation    | float  | 旋转角度     | 50.2                           |
| ├─ obj_type   | int    | 物品类型     | 1.机器人 2.可交换塑料箱 3.传送带 4.障碍物架子   |
| ├─ position    | array  | 坐标位置         | ["x":"12.22","y":"12.22","z":"12.22"] |
| ├─ size        | array  | 大小（长宽高 ）     | ["length":"1","wide":"2","high":"3"]  |

示例
``` json
{
    "code": 200,
    "message": "success",
    "data": {
                "ability_code": [
                    "move",
                    "grab",
                    "navigate",
                    "sense"
                ],
                "ability_name": [
                    "移动",
                    "抓取",
                    "导航",
                    "感知"
                ],
                "describe": "PR2双臂服务机器人，具备移动和操作能力",
                "id": 1,
                "name": "PR2 Robot",
                "rotation": 50.2,  
                "obj_type": 1,  
                "position": {
                    "x": "-2.50",
                    "y": "-3.50",
                    "z": "-0.03"
                },
               "size": ["length":"1","wide":"2","high":"3"]
    }
}
```


## 获取场景的单个物品移动坐标

### 请求地址
`GET /api/v1/scene/coordinate`

`请求url格式例子：/api/v1/scene/coordinate?id=path_planning&object_id=1&position=["x":"111", "y":"111"]`

### 请求参数
| 字段名       | 类型    | 必填 | 说明         | 示例值                    |
|-----------|-------|------|------------|------------------------|
| id        | int   | 是 | 场景ID       | 1                      |
| object_id | int   | 是 | 物品id       | 1                      |
| position  | array | 是 | 目标位置的x,y坐标 | ["x":"111", "y":"111"] |

### 响应参数
| 字段名         | 类型     | 说明                                | 示例值                               |
|-------------|--------|-----------------------------------|-----------------------------------|
| code        | int    | 状态码                               | 200                               |
| message     | string | 消息                                | "success"                         |
| data        | object | 数据                                | null                              |
| ├─ id       | int    | 物品id                              | 1                                 |
| ├─ position | string | 物品位置的x,y,z坐标                      | ["x":"111", "y":"111", "z":"111"] |
| ├─ status   | int    | 移动状态（1.未移动，2.移动中，3.移动结束（已移动到目标点）） | 1                                 |

示例
``` json
{
    "code": 200,
    "message": "success",
    "data": {
        "id": 1,
        "status":3,
        "position": {
            "x": "-2.50",
            "y": "-3.50",
            "z": "-0.03"
        }
}
```

## 轮臂机器人移动命令下发

### 请求地址
`POST /api/v1/move/set_goal`

### 请求参数
| 字段名 | 类型 | 必填 | 说明 | 示例值 |
| ------ | ---- | ---- | ---- | ------ |
| id        | int64 | 是 | 场景id，需与当前已加载场景id一致，否则返回报错 | 1 |
| object_id | int64 | 是 | 要移动的机器人的物品id，在操作的机器人没有移动能力时返回报错 | 1 |
| goal_pos   | list    | 是 |目标点的行、列坐标，可以是`range(0, 8)`的浮点数，在目标不可达时返回报错         | [2,6]            |
| goal_angle | float | 是 | 机器人与目标点的目标偏航角，角度制，-180°到180°之间的浮点数 | 1 |


### 响应参数
| 字段名         | 类型   | 说明           | 示例值                                   |
| -------------- | ------ | -------------- | ---------------------------------------- |
| code           | int    | 状态码         | 200                                      |
| message        | string | 消息           | "success"                                |
| is_reachable        | bool | 给定的目标地点是否可达           | true                                |
| total_segments        | int | 规划得到的路径的总段数           | 5                               |

示例
``` json
{
    "code": 200,
    "message": "success",
    "is_reachable": true,
    "total_segments": 5
}

{
    "code":400,
    "message": "Target is unreachable",
    "is_reachable": false,
    "total_segments": 0
}
```
## 轮臂机器人移动状态获取

### 请求地址
`GET /api/v1/move/check_state`

### 请求参数
| 字段名 | 类型 | 必填 | 说明                                          | 示例值 |
| ------ | ---- | ---- | --------------------------------------------- | ------ |
| id        | int64 | 是 | 场景id，需与当前已加载场景id一致，否则返回报错 | 1 |
| object_id | int64 | 是 | 要移动的机器人的物品id，在该机器人没收到移动命令时返回报错 | 1 |

### 响应参数
| 字段名  | 类型   | 说明   | 示例值    |
| ------- | ------ | ------ | --------- |
| code    | int    | 状态码 | 200       |
| message | string | 消息   | "success" |
| status   | int    | 移动状态（1.未移动，2.移动中，3.移动结束（已移动到目标点）） | 1                                 |
| process | string | 机器人移动进度   | "4/5"，表示共有5步，当前已完成四步，正在执行第五步 |


示例
``` json
{
    "code": 200,
    "message": "success",
    "status": 2,
    "process": "1/5"
}
```
## 获取场景的单个物品移动坐标

### 请求地址
`GET /api/v1/move/check_pos`

### 请求参数
| 字段名    | 类型  | 必填 | 说明              | 示例值                 |
| --------- | ----- | ---- | ----------------- | ---------------------- |
| id        | int   | 是   | 场景ID            | 1                      |
| object_id | int   | 是   | 物品id            | 1                      |

### 响应参数
| 字段名      | 类型   | 说明                                                         | 示例值                            |
| ----------- | ------ | ------------------------------------------------------------ | --------------------------------- |
| code        | int    | 状态码                                | 200                               |
| message     | string | 消息                                  | "success"                         |
| data        | object | 数据                            | null                              |
| ├─ id       | int    | 物品id                        1                                 |
| ├─ position | string | 物品位置的x,y,z坐标（单位是m）               | ["x":"111", "y":"111", "z":"111"] |
| ├─ orien   | string    | 物品位置的x,y,z轴旋转角（角度制，单位是度） | ["rx":"111", "ry":"111", "rz":"111"]   |

示例
``` json
{
    "code": 200,
    "message": "success",
    "data": {
        "id": 1,
        "orien":{
            "rx": "90.00",
            "ry": "91.23",
            "rz": "0.00"
        },
        "position": {
            "x": "-2.50",
            "y": "-3.50",
            "z": "-0.03"
        }
    }
}
```

## 轮臂机器人抓取并放置命令

### 请求地址

`POST /api/v1/capture/pick_and_place`

### 说明

### 请求参数

| 字段名         | 类型        | 必填 | 默认值                                   | 说明                                                         | 示例值                |
| -------------- | ----------- | ---- | ---------------------------------------- | ------------------------------------------------------------ | --------------------- |
| id             | int         | 是   |                                          | 场景id，需与当前已加载场景id一致，否则返回报错               | 1                     |
| robot_id       | int         | 是   |                                          | 机器人的id，在操作的机器人没有抓取能力时返回报错             | 1                     |
| goal_arm       | str         | 否   | both                                     | 规划左手："left"，右手："right"，双手:"both"                 | bleft_oth             |
| left_cur_pos   | list[float] | 否   | [-0.01749985 , 0.29927  ,  -0.21073727]  | 抓取时左手在的位置，xyz 坐标,goal_arm为right时此参数不会被使用 | [0.24  ,0.388 , 0.2]  |
| left_goal_pos  | list[float] | 否   | [-0.01749985 , 0.29927  ,  -0.21073727]  | 要将左手移动到的目标位置,内容为xyz 坐标,goal_arm为right时此参数不会被使用 | [1.0,2.0,3.0]         |
| left_angle     | list[float] | 否   | [-3.02456926, -0.00675474,  0.09522905]  | 抓取时左手在的姿态,整个抓取过程中保持不变,goal_arm为right时此参数不会被使用 | [-3.14,0,0]           |
| right_cur_pos  | list[float] | 否   | [-0.01749985 ,-0.29927  ,  -0.21073727]  | 抓取时右手在的位置，xyz 坐标,goal_arm为left 时此参数不会被使用 | [0.24  ,-0.388 , 0.2] |
| right_goal_pos | list[float] | 否   | [-0.01749985 , -0.29927  ,  -0.21073727] | 要将右手移动到的目标位置,内容为xyz 坐标,goal_arm为left 时此参数不会被使用 | [1.0,2.0,3.0]         |
| right_angle    | list[float] | 否   | [3.14,0,0]                               | 抓取时右手在的姿态,整个抓取过程中保持不变,goal_arm为left 时此参数不会被使用 | [-3.14,0,0]           |


### 响应参数

| 字段名         | 类型        | 说明                                              | 示例值        |
| -------------- | ----------- | ------------------------------------------------- | ------------- |
| code           | int         | 状态码                                            | 200           |
| message        | string      | 消息                                              | "success"     |
| left_real_pos  | list[float] | 左手实际移动到的位置，可能会和left_goal_pos有误差 | [1.0,2.0.3.0] |
| right_real_pos | list[float] | 右手实际移动到的位置，可能会和goal_pos有误差      | [1.0,2.0.3.0] |

示例

``` json
{
    "code": 200,
    "message": "success",
    "left_real_pos":[1.0,2.0,3.0],
    "right_real_pos":[1.0,2.0.3.0]
}

{
    "code":400,
    "message": "The target for capture cannot be reached."
}
```

## 轮臂机器人抓取命令

### 请求地址

`POST /api/v1/capture/pick

### 请求参数

| 字段名        | 类型        | 必填 | 默认值                                  | 说明                                                         | 示例值                |
| ------------- | ----------- | ---- | --------------------------------------- | ------------------------------------------------------------ | --------------------- |
| id            | int         | 是   |                                         | 场景id，需与当前已加载场景id一致，否则返回报错               | 1                     |
| robot_id      | int         | 是   |                                         | 机器人的id，在操作的机器人没有抓取能力时返回报错             | 1                     |
| goal_arm      | str         | 否   | both                                    | 规划左手："left"，右手："right"，双手:"both"                 | bleft_oth             |
| left_cur_pos  | list[float] | 否   | [-0.01749985 , 0.29927  ,  -0.21073727] | 抓取时左手在的位置，xyz 坐标,goal_arm为right时此参数不会被使用 | [0.24  ,0.388 , 0.2]  |
| left_angle    | list[float] | 否   | [-3.02456926, -0.00675474,  0.09522905] | 抓取时左手在的姿态,整个抓取过程中保持不变,goal_arm为right时此参数不会被使用 | [-3.14,0,0]           |
| right_cur_pos | list[float] | 否   | [0.2474993,-0.33825069,0.12093117]      | 抓取时右手在的位置，xyz 坐标,goal_arm为left 时此参数不会被使用 | [0.24  ,-0.388 , 0.2] |
| right_angle   | list[float] | 否   | [3.14,0,0]                              | 抓取时右手在的姿态,整个抓取过程中保持不变,goal_arm为left 时此参数不会被使用 | [-3.14,0,0]           |

### 响应参数

| 字段名         | 类型        | 说明                                              | 示例值        |
| -------------- | ----------- | ------------------------------------------------- | ------------- |
| code           | int         | 状态码                                            | 200           |
| message        | string      | 消息                                              | "success"     |
| left_real_pos  | list[float] | 左手实际移动到的位置，可能会和left_goal_pos有误差 | [1.0,2.0.3.0] |
| right_real_pos | list[float] | 右手实际移动到的位置，可能会和goal_pos有误差      | [1.0,2.0.3.0] |

示例

``` json
{
    "code": 200,
    "message": "success",
    "left_real_pos":[1.0,2.0,3.0],
    "right_real_pos":[1.0,2.0.3.0]
}

{
    "code":400,
    "message": "The target for capture cannot be reached."
}
```

## 轮臂机器人放置命令

### 请求地址

`POST /api/v1/capture/place

### 请求参数

| 字段名         | 类型        | 必填 | 默认值                                   | 说明                                                         | 示例值        |
| -------------- | ----------- | ---- | ---------------------------------------- | ------------------------------------------------------------ | ------------- |
| id             | int         | 是   |                                          | 场景id，需与当前已加载场景id一致，否则返回报错               | 1             |
| robot_id       | int         | 是   |                                          | 机器人的id，在操作的机器人没有抓取能力时返回报错             | 1             |
| goal_arm       | str         | 否   | both                                     | 规划左手："left"，右手："right"，双手:"both"                 | bleft_oth     |
| left_goal_pos  | list[float] | 否   | [-0.01749985 , 0.29927  ,  -0.21073727]  | 要将左手移动到的目标位置,内容为xyz 坐标,goal_arm为right时此参数不会被使用 | [1.0,2.0,3.0] |
| left_angle     | list[float] | 否   | [-3.02456926, -0.00675474,  0.09522905]  | 抓取时左手在的姿态,整个抓取过程中保持不变,goal_arm为right时此参数不会被使用 | [-3.14,0,0]   |
| right_goal_pos | list[float] | 否   | [-0.01749985 , -0.29927  ,  -0.21073727] | 要将右手移动到的目标位置,内容为xyz 坐标,goal_arm为left 时此参数不会被使用 | [1.0,2.0,3.0] |
| right_angle    | list[float] | 否   | [3.14,0,0]                               | 抓取时右手在的姿态,整个抓取过程中保持不变,goal_arm为left 时此参数不会被使用 | [-3.14,0,0]   |


### 响应参数

| 字段名         | 类型        | 说明                                              | 示例值        |
| -------------- | ----------- | ------------------------------------------------- | ------------- |
| code           | int         | 状态码                                            | 200           |
| message        | string      | 消息                                              | "success"     |
| left_real_pos  | list[float] | 左手实际移动到的位置，可能会和left_goal_pos有误差 | [1.0,2.0.3.0] |
| right_real_pos | list[float] | 右手实际移动到的位置，可能会和goal_pos有误差      | [1.0,2.0.3.0] |

示例

``` json
{
    "code": 200,
    "message": "success",
    "left_real_pos":[1.0,2.0,3.0],
    "right_real_pos":[1.0,2.0.3.0]
}

{
    "code":400,
    "message": "The target for capture cannot be reached."
}
```

## 获取抓取命令结果

### 请求地址

`POST /api/v1/capture/pick_result

### 请求参数

| 字段名    | 类型 | 说明                                           | 示例值 |
| --------- | ---- | ---------------------------------------------- | ------ |
| id        | int  | 场景id，需与当前已加载场景id一致，否则返回报错 | 1      |
| robot_id  | int  | 机器人的id                                     | 1      |
| object_id | int  | 物品id                                         | 2      |

### 说明

抓取任务结束则返回true，否则返回false;只可以调用一次，得到true结果后程序会重置结果为false

### 响应参数

| 字段名  | 类型 | 说明   | 示例值 |
| ------- | ---- | ------ | ------ |
| code    | int  | 状态码 | 200    |
| message | bool | 消息   | true   |

示例

``` json
{
    "code": 200,
    "message": false
}

{
    "code": 200,
    "message": true
}
```

## 获取放置命令结果

### 请求地址

`POST /api/v1/capture/place_result

### 请求参数

| 字段名    | 类型 | 说明                                           | 示例值 |
| --------- | ---- | ---------------------------------------------- | ------ |
| id        | int  | 场景id，需与当前已加载场景id一致，否则返回报错 | 1      |
| robot_id  | int  | 机器人的id                                     | 1      |
| object_id | int  | 物品id                                         | 2      |

### 说明

放置任务结束则返回true，否则返回false，只可以调用一次;得到true结果后程序会重置结果为false

### 响应参数

| 字段名  | 类型 | 说明   | 示例值 |
| ------- | ---- | ------ | ------ |
| code    | int  | 状态码 | 200    |
| message | bool | 消息   | true   |

示例

``` json
{
    "code": 200,
    "message": false
}

{
    "code": 200,
    "message": true
}
```



## 状态码说明
| 状态码  | 说明 |
|------|------|
| 200  | 请求成功 |