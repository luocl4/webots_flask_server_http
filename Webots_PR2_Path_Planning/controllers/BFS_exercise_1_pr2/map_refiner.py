# map_refiner.py
# 地图精细化工具 - 将world_map扩展到更高分辨率

import numpy as np
import json
import time
from typing import List, Tuple, Optional
import math

class MapRefiner:
    """地图精细化类，支持将低分辨率地图扩展为高分辨率地图"""
    
    def __init__(self, scale_factor: int = 100):
        """
        初始化地图精细化器
        
        Args:
            scale_factor: 缩放因子，默认100倍
        """
        self.scale_factor = scale_factor
        self.original_map = None
        self.refined_map = None
        self.map_size = None
    
    def refine_world_map(self, world_map: List[List[int]]) -> List[List[int]]:
        """
        精细化世界地图 - 将地图大小扩展到原本的scale_factor倍
        
        Args:
            world_map: 原始世界地图 (0=空地, 1=障碍物)
        
        Returns:
            精细化后的地图
        """
        self.original_map = world_map
        original_height = len(world_map)
        original_width = len(world_map[0])
        self.map_size = (original_height, original_width)
        
        # 计算新地图尺寸
        new_height = original_height * self.scale_factor
        new_width = original_width * self.scale_factor
        
        print(f"地图精细化")
        
        # 创建精细化地图
        refined_map = [[0 for _ in range(new_width)] for _ in range(new_height)]
        
        # 逐像素扩展
        for i in range(original_height):
            for j in range(original_width):
                value = world_map[i][j]
                
                start_row = i * self.scale_factor
                end_row = start_row + self.scale_factor
                start_col = j * self.scale_factor
                end_col = start_col + self.scale_factor
                
                for row in range(start_row, end_row):
                    for col in range(start_col, end_col):
                        refined_map[row][col] = value
        
        self.refined_map = refined_map
        
        return refined_map
    
    def convert_coordinates(self, pos):
        """
        将Webots世界坐标系中的位置或原始地图坐标转换为精细化地图坐标
        
        Args:
            pos: 可以是以下两种类型：
                - Webots中的位置 [x, y] 或 [x, y, z]，当为三维坐标时只取x和y
                - 原始地图中的位置 (row, col)
        
        Returns:
            精细化地图中对应的位置 (row, col)
        """
        if self.scale_factor is None:
            raise ValueError("请先调用 refine_world_map 方法")
        
        # 检查是否为列表类型（可能是从GPS获取的Webots坐标）
        if isinstance(pos, list) or (hasattr(pos, '__iter__') and not isinstance(pos, tuple)):
            if len(pos) >= 2:
                # 只取前两个坐标 (x, y)
                x, y = pos[0], pos[1]
                
                # Webots坐标系通常是x前进方向，y左右方向
                # 转换为地图坐标系（行列）
                col = int(x * self.scale_factor + (self.scale_factor * self.map_size[1] // 2))
                row = int((self.scale_factor * self.map_size[0] // 2) - y * self.scale_factor)
                
                return (int(row), int(col))
            else:
                raise ValueError("Webots坐标至少需要包含x和y两个维度")
        
        # 如果是元组，假设是原始地图坐标 (row, col)
        elif isinstance(pos, tuple) and len(pos) == 2:
            row, col = pos
            # 转换到精细化地图的中心位置
            new_row = int(row * self.scale_factor + self.scale_factor // 2)
            new_col = int(col * self.scale_factor + self.scale_factor // 2)
            
            return (int(new_row), int(new_col))
        
        else:
            raise ValueError("不支持的坐标格式，需要是[x, y]或(row, col)格式")
    
    def convert_coordinates_truth(self, pos):
        """
        将Webots世界坐标系中的位置转换为精细化地图坐标
        
        Args:
            pos: Webots中的位置 (x, y) 或 (x, y, z)，当为三维坐标时只取x和y
        
        Returns:
            精细化地图中对应的位置 (row, col)
        """
        if self.scale_factor is None:
            raise ValueError("请先调用 refine_world_map 方法")
        
        # 处理传入的坐标，支持二维和三维坐标
        if len(pos) >= 2:
            # 只取前两个坐标 (x, y)
            x, y = pos[0], pos[1]
        else:
            raise ValueError("坐标至少需要包含x和y两个维度")
            
        # Webots坐标系通常是x前进方向，y左右方向
        # 转换为地图坐标系（行列）
        col = int(x * self.scale_factor + (self.scale_factor * self.map_size[1] // 2))
        row = int((self.scale_factor * self.map_size[0] // 2) - y * self.scale_factor)
        
        return (int(row), int(col))

    def convert_coordinates_back(self, refined_pos: Tuple[int, int]) -> Tuple[int, int]:
        """
        将精细化地图坐标转换回原始地图坐标
        
        Args:
            refined_pos: 精细化地图中的位置 (row, col)
        
        Returns:
            原始地图中对应的位置
        """
        if self.scale_factor is None:
            raise ValueError("请先调用 refine_world_map 方法")
        
        row, col = refined_pos
        # 转换回原始地图坐标
        original_row = int(row // self.scale_factor)
        original_col = int(col // self.scale_factor)
        
        return (int(original_row), int(original_col))
    
    def save_refined_map(self, filename: str, format_type: str = "json") -> None:
        """
        保存精细化地图到文件
        
        Args:
            filename: 文件名
            format_type: 文件格式 ("json", "txt", "npy")
        """
        if self.refined_map is None:
            raise ValueError("没有精细化地图可保存")
        
        if format_type == "json":
            with open(filename, 'w') as f:
                json.dump({
                    "refined_map": self.refined_map,
                    "scale_factor": self.scale_factor,
                    "original_size": self.map_size,
                    "refined_size": (len(self.refined_map), len(self.refined_map[0]))
                }, f, indent=2)
        
        elif format_type == "txt":
            with open(filename, 'w') as f:
                f.write(f"# 精细化地图 - 缩放因子: {self.scale_factor}x\n")
                f.write(f"# 原始尺寸: {self.map_size}\n")
                f.write(f"# 精细化尺寸: {len(self.refined_map)}x{len(self.refined_map[0])}\n")
                for row in self.refined_map:
                    f.write(' '.join(map(str, row)) + '\n')
        
        elif format_type == "npy":
            np.save(filename, np.array(self.refined_map))
        
        print(f"💾 精细化地图已保存至: {filename}")

def inflate_obstacles(map_data, inflation_radius):
    """
    对地图中的障碍物进行膨胀处理，为障碍物周围添加安全区域
    
    Args:
        map_data: 原始地图数据 (0=空地, 1=障碍物)
        inflation_radius: 膨胀半径，以像素/格子为单位
        
    Returns:
        膨胀后的地图
    """
    if inflation_radius <= 0:
        return map_data
    
    map_height = len(map_data)
    map_width = len(map_data[0]) if map_height > 0 else 0
    
    # 创建副本，避免修改原始地图
    inflated_map = [[0 for _ in range(map_width)] for _ in range(map_height)]
    
    # 复制原始障碍物
    for i in range(map_height):
        for j in range(map_width):
            if map_data[i][j] == 1:
                inflated_map[i][j] = 1
    
    # 查找所有障碍物
    obstacles = []
    for i in range(map_height):
        for j in range(map_width):
            if map_data[i][j] == 1:
                obstacles.append((i, j))
    
    # 对每个障碍物进行膨胀
    for obstacle in obstacles:
        obs_i, obs_j = obstacle
        
        # 在障碍物周围创建安全区域 - 转换浮点数膨胀半径为整数
        int_radius = int(inflation_radius)  # 向下取整
        float_remainder = inflation_radius - int_radius  # 余数部分
        
        # 使用整数部分进行主要扩展
        for i in range(max(0, obs_i - int_radius), min(map_height, obs_i + int_radius + 1)):
            for j in range(max(0, obs_j - int_radius), min(map_width, obs_j + int_radius + 1)):
                # 计算到障碍物的距离
                distance = math.sqrt((i - obs_i)**2 + (j - obs_j)**2)
                
                # 如果在膨胀半径内，标记为障碍物
                if distance <= inflation_radius:
                    inflated_map[i][j] = 1
    
    return inflated_map
