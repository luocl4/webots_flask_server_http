import os
import trimesh
from pathlib import Path
import argparse

def convert_stl_to_obj(stl_dir, obj_dir, recursive=False, verbose=False):
    """将STL文件转换为OBJ格式（支持大小写不敏感）"""
    # 创建输出目录
    os.makedirs(obj_dir, exist_ok=True)
    
    print(f"\n=== STL转OBJ转换器 ===")
    print(f"源目录: {stl_dir}")
    print(f"目标目录: {obj_dir}")
    print(f"递归搜索: {'是' if recursive else '否'}")
    
    # 收集STL文件（大小写不敏感）
    if recursive:
        stl_files = list(Path(stl_dir).rglob("*.STL")) + list(Path(stl_dir).rglob("*.stl"))
    else:
        stl_files = list(Path(stl_dir).glob("*.STL")) + list(Path(stl_dir).glob("*.stl"))
    
    if not stl_files:
        print(f"⚠️ 警告: 在 {stl_dir} 中未找到STL文件!")
        if os.path.exists(stl_dir):
            print(f"  目录内容:")
            for item in os.listdir(stl_dir):
                print(f"  - {item}")
        return
    
    print(f"找到 {len(stl_files)} 个STL文件")
    
    # 转换文件
    success_count = 0
    for stl_file in stl_files:
        try:
            rel_path = stl_file.relative_to(stl_dir).parent
            output_subdir = Path(obj_dir) / rel_path
            os.makedirs(output_subdir, exist_ok=True)
            
            obj_file = output_subdir / f"{stl_file.stem}.obj"
            
            if verbose:
                print(f"正在转换: {stl_file}")
            
            mesh = trimesh.load_mesh(str(stl_file))
            mesh.export(str(obj_file), file_type="obj")
            
            print(f"✅ 成功: {stl_file.name} -> {obj_file.name}")
            success_count += 1
            
        except Exception as e:
            print(f"❌ 失败: {stl_file.name} - {str(e)}")
    
    print(f"\n转换完成! 成功: {success_count}, 失败: {len(stl_files) - success_count}")
    print(f"OBJ文件保存在: {obj_dir}")

def main():
    parser = argparse.ArgumentParser(description="STL到OBJ格式转换器")
    parser.add_argument("-s", "--source", required=True, help="STL文件所在目录")
    parser.add_argument("-o", "--output", required=True, help="OBJ文件输出目录")
    parser.add_argument("-r", "--recursive", action="store_true", help="递归搜索子目录")
    parser.add_argument("-v", "--verbose", action="store_true", help="显示详细信息")
    
    args = parser.parse_args()
    convert_stl_to_obj(args.source, args.output, args.recursive, args.verbose)

if __name__ == "__main__":
    main()
