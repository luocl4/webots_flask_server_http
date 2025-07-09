#!/usr/bin/env python3
# wbt_parser.py - Webots世界文件(.wbt)解析工具

import os
import json

def extract_label_points_from_wbt(wbt_file_path):
    """从Webots的.wbt文件中提取label点的坐标
    
    Args:
        wbt_file_path: .wbt文件的完整路径
        
    Returns:
        成功时返回字典列表，每个字典包含:
        - name: label的名称
        - position: {x,y,z} 坐标
        失败时返回空列表
    """
    # 调用综合解析函数
    result = parse_wbt_file(wbt_file_path)
    return result['label_points']

def extract_floor_size_from_wbt(wbt_file_path):
    """从Webots的.wbt文件中提取floorSize字段的值
    
    Args:
        wbt_file_path: .wbt文件的完整路径
        
    Returns:
        成功时返回地图四个角的坐标列表，每个角是一个字典 {'x':x, 'y':y, 'z':z}
        失败时返回 None
    """
    # 调用综合解析函数
    result = parse_wbt_file(wbt_file_path)
    return result['floor_corners']

def update_scene_configs_with_label_points(scene_configs):
    """更新场景配置，为每个场景添加label_point和floor_corners信息
    
    Args:
        scene_configs: 场景配置字典
        
    Returns:
        更新后的场景配置字典
    """
    for scene_id, config in scene_configs.items():
        if 'working_dir' in config and 'world_file' in config:
            wbt_file_path = os.path.join(config['working_dir'], config['world_file'])
            print(f"正在解析 {wbt_file_path} 中的信息...")
            
            # 解析wbt文件
            parse_result = parse_wbt_file(wbt_file_path)
            label_points = parse_result['label_points']
            floor_corners = parse_result['floor_corners']
            
            # 更新标签点信息
            if label_points:
                # 将label_points添加到场景配置中
                config['label_point'] = []  # 清空之前的数据
                    
                for point in label_points:
                    config['label_point'].append({
                        point['name']: point['position']
                    })
                    
                print(f"为场景 '{scene_id}' 添加了 {len(label_points)} 个标签点")
            else:
                config['label_point'] = []  # 确保即使没有找到标签点也有空列表
                print(f"场景 '{scene_id}' 中未找到标签点")
            
            # 更新地图尺寸信息
            if floor_corners:
                config['floor_corners'] = floor_corners
                print(f"为场景 '{scene_id}' 添加了地图尺寸信息")
    
    return scene_configs

def parse_wbt_file(wbt_file_path):
    """从Webots的.wbt文件中提取信息，包括floorSize和label点坐标
    
    Args:
        wbt_file_path: .wbt文件的完整路径
        
    Returns:
        返回包含解析信息的字典:
        - floor_corners: 地图四个角的坐标列表 [{'x':x1, 'y':y1, 'z':z1}, ...]
        - label_points: 标签点列表 [{'name': name1, 'position': {'x':x1, 'y':y1, 'z':z1}}, ...]
        如果解析失败，相应的字段值为None或空列表
    """
    result = {
        'floor_corners': None,
        'label_points': []
    }
    
    try:
        if not os.path.exists(wbt_file_path):
            print(f"解析wbt文件错误: 文件不存在 - {wbt_file_path}")
            return result
        
        # 读取整个文件内容
        with open(wbt_file_path, 'r', encoding='utf-8') as file:
            content = file.read()
            
        lines = content.split('\n')
        
        # 提取floorSize信息
        floor_size = None
        for line in lines:
            line = line.strip()
            if 'floorSize' in line:
                parts = line.split()
                if len(parts) >= 3 and parts[0] == 'floorSize':
                    try:
                        width = float(parts[1])
                        height = float(parts[2])
                        floor_size = (width, height)
                        break
                    except ValueError:
                        print(f"提取地图尺寸错误: 无法解析数值 - {line}")
        
        # 计算地图四个角坐标
        if floor_size:
            width, height = floor_size
            half_width = width / 2
            half_height = height / 2
            
            result['floor_corners'] = [
                {'x':half_width, 'y':half_height, 'z':0},    # 右上角
                {'x':-half_width, 'y':-half_height, 'z':0},  # 左下角
                {'x':half_width, 'y':-half_height, 'z':0},   # 右下角
                {'x':-half_width, 'y':half_height, 'z':0}    # 左上角
            ]
            print(f"成功提取地图尺寸: {width} x {height}")
        
        # 提取label点信息
        i = 0
        label_points = []
        
        while i < len(lines):
            line = lines[i].strip()
            
            # 寻找label节点的开始
            if line.startswith('DEF') and 'label' in line.lower() and 'Solid' in line:
                # 提取DEF名称
                parts = line.split()
                if len(parts) >= 3:
                    label_name = parts[1]
                    
                    # 向前查找translation
                    translation = None
                    node_name = None
                    open_braces = 1  # 已经在第一个 Solid { 里面了
                    
                    # 从下一行开始查找translation和name
                    j = i + 1
                    while j < len(lines) and open_braces > 0:
                        current_line = lines[j].strip()
                        
                        # 计算括号平衡
                        open_braces += current_line.count('{')
                        open_braces -= current_line.count('}')
                        
                        # 查找translation
                        if 'translation' in current_line and translation is None:
                            trans_parts = current_line.split('translation')[1].strip().split()
                            if len(trans_parts) >= 3:
                                try:
                                    translation = {
                                        'x': float(trans_parts[0]),
                                        'y': float(trans_parts[1]),
                                        'z': float(trans_parts[2])
                                    }
                                except ValueError:
                                    print(f"无法解析translation值: {current_line}")
                        
                        # 查找name
                        if 'name' in current_line and '"' in current_line:
                            try:
                                name_match = current_line.split('"')
                                if len(name_match) >= 3:
                                    node_name = name_match[1]
                            except Exception:
                                pass
                        
                        j += 1
                    
                    # 如果找到了translation，添加到结果中
                    if translation:
                        # 如果找到了name，优先使用name
                        final_name = node_name if node_name else label_name
                        
                        label_points.append({
                            'name': final_name,
                            'position': translation
                        })
                        print(f"找到标签点: {final_name} at {translation}")
                
                # 跳过已处理的节点
                i = j
            else:
                i += 1
        
        result['label_points'] = label_points
        
        return result
        
    except Exception as e:
        print(f"解析wbt文件时发生异常: {str(e)}")
        import traceback
        traceback.print_exc()
        return result

if __name__ == "__main__":
    # 测试代码
    test_file = "./Webots_PR2_Path_Planning/worlds/path_planning.wbt"
    if os.path.exists(test_file):
        # 测试parse_wbt_file函数
        result = parse_wbt_file(test_file)
        
        print("\n===== 地图尺寸信息 =====")
        if result['floor_corners']:
            print(f"找到地图四个角点坐标:")
            for i, corner in enumerate(result['floor_corners']):
                corner_name = ["右上角", "左下角", "右下角", "左上角"][i]
                print(f"  - {corner_name}: x={corner['x']}, y={corner['y']}, z={corner['z']}")
        else:
            print("未找到地图尺寸信息")
            
        print("\n===== 标签点信息 =====")
        if result['label_points']:
            print(f"找到 {len(result['label_points'])} 个标签点:")
            for point in result['label_points']:
                print(f"  - {point['name']}: x={point['position']['x']}, y={point['position']['y']}, z={point['position']['z']}")
        else:
            print("未找到标签点")
    else:
        print(f"测试文件不存在: {test_file}")
