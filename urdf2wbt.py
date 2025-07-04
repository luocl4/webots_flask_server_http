import os
from urdf2webots.importer import convertUrdfFile

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(current_dir, 'v4_pro_arm', 'urdf', 'urdf_arm.urdf')
    output_file = os.path.join(current_dir, 'v4_pro_arm', 'urdf_arm.proto')
    params = {
        'input': input_file,
        'output': output_file,
        'boxCollision': True,
        'normal': True,
        'targetVersion': 'R2023b',
        'initRotation': '0 0 1 0',  # 设置默认旋转
        'toolSlot': None  # 不使用工具槽
    }
    
    convertUrdfFile(**params)