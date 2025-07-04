#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件上传工具
用于通过 multipart/form-data 格式上传文件到服务器
"""

import requests
import os
import sys
from pathlib import Path
import json
from typing import Optional, Dict, Any


class FileUploader:
    """文件上传器"""
    
    def __init__(self, base_url: str = "http://clusters.luo980.com:13012"):
        """
        初始化文件上传器
        
        Args:
            base_url: 服务器基础URL
        """
        self.base_url = base_url.rstrip('/')
        self.upload_endpoint = f"{self.base_url}/api/upload/file/2"
    
    def upload_file(self, file_path: str, custom_filename: Optional[str] = None) -> Dict[str, Any]:
        """
        上传文件到服务器
        
        Args:
            file_path: 要上传的文件路径
            custom_filename: 自定义文件名（可选）
            
        Returns:
            服务器响应的字典，包含 code, message, data 字段
            
        Raises:
            FileNotFoundError: 文件不存在
            requests.RequestException: 网络请求异常
        """
        # 检查文件是否存在
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"文件不存在: {file_path}")
        
        # 获取文件信息
        file_path = Path(file_path)
        filename = custom_filename if custom_filename else file_path.name
        
        print(f"正在上传文件: {file_path}")
        print(f"文件名: {filename}")
        print(f"文件大小: {file_path.stat().st_size / 1024:.2f} KB")
        print(f"上传地址: {self.upload_endpoint}")
        
        try:
            # 准备文件数据
            with open(file_path, 'rb') as f:
                files = {
                    'file': (filename, f, self._get_content_type(file_path))
                }
                
                # 发送POST请求
                response = requests.post(
                    self.upload_endpoint,
                    files=files,
                    timeout=30  # 30秒超时
                )
                
                # 检查HTTP状态码
                response.raise_for_status()
                
                # 解析JSON响应
                result = response.json()
                
                # 打印结果
                print(f"上传完成!")
                print(f"状态码: {result.get('code', 'unknown')}")
                print(f"消息: {result.get('message', 'unknown')}")
                
                if result.get('data'):
                    print(f"文件链接: {result['data']}")
                
                return result
                
        except requests.exceptions.Timeout:
            raise requests.RequestException("上传超时，请检查网络连接")
        except requests.exceptions.ConnectionError:
            raise requests.RequestException(f"无法连接到服务器: {self.upload_endpoint}")
        except requests.exceptions.HTTPError as e:
            raise requests.RequestException(f"HTTP错误: {e}")
        except json.JSONDecodeError:
            raise requests.RequestException("服务器返回无效的JSON格式")
    
    def _get_content_type(self, file_path: Path) -> str:
        """
        根据文件扩展名获取Content-Type
        
        Args:
            file_path: 文件路径
            
        Returns:
            MIME类型字符串
        """
        ext = file_path.suffix.lower()
        content_types = {
            '.txt': 'text/plain',
            '.pdf': 'application/pdf',
            '.doc': 'application/msword',
            '.docx': 'application/vnd.openxmlformats-officedocument.wordprocessingml.document',
            '.xls': 'application/vnd.ms-excel',
            '.xlsx': 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet',
            '.ppt': 'application/vnd.ms-powerpoint',
            '.pptx': 'application/vnd.openxmlformats-officedocument.presentationml.presentation',
            '.jpg': 'image/jpeg',
            '.jpeg': 'image/jpeg',
            '.png': 'image/png',
            '.gif': 'image/gif',
            '.mp4': 'video/mp4',
            '.avi': 'video/avi',
            '.mov': 'video/quicktime',
            '.mp3': 'audio/mpeg',
            '.wav': 'audio/wav',
            '.zip': 'application/zip',
            '.rar': 'application/rar',
            '.json': 'application/json',
            '.xml': 'application/xml',
            '.csv': 'text/csv',
        }
        return content_types.get(ext, 'application/octet-stream')
    
    def batch_upload(self, file_paths: list, custom_filenames: Optional[list] = None) -> list:
        """
        批量上传文件
        
        Args:
            file_paths: 文件路径列表
            custom_filenames: 自定义文件名列表（可选）
            
        Returns:
            上传结果列表
        """
        results = []
        total_files = len(file_paths)
        
        for i, file_path in enumerate(file_paths, 1):
            custom_name = None
            if custom_filenames and i <= len(custom_filenames):
                custom_name = custom_filenames[i-1]
            
            print(f"\n[{i}/{total_files}] 上传文件: {file_path}")
            
            try:
                result = self.upload_file(file_path, custom_name)
                results.append({
                    'file_path': file_path,
                    'success': True,
                    'result': result
                })
            except Exception as e:
                print(f"上传失败: {str(e)}")
                results.append({
                    'file_path': file_path,
                    'success': False,
                    'error': str(e)
                })
        
        return results


def main():
    """主函数 - 命令行使用示例"""
    if len(sys.argv) < 2:
        print("使用方法:")
        print(f"  {sys.argv[0]} <文件路径>")
        print(f"  {sys.argv[0]} <文件路径> <自定义文件名>")
        print(f"  {sys.argv[0]} <文件路径1> <文件路径2> ...")
        print()
        print("示例:")
        print(f"  {sys.argv[0]} test.txt")
        print(f"  {sys.argv[0]} test.txt my_file.txt")
        print(f"  {sys.argv[0]} file1.jpg file2.mp4 document.pdf")
        sys.exit(1)
    
    # 创建上传器实例
    uploader = FileUploader()
    
    # 获取文件路径
    file_paths = sys.argv[1:]
    
    try:
        if len(file_paths) == 1:
            # 单文件上传
            result = uploader.upload_file(file_paths[0])
            if result.get('code') == 200:
                print("\n✅ 上传成功!")
            else:
                print(f"\n❌ 上传失败: {result.get('message', '未知错误')}")
        else:
            # 多文件上传
            print(f"开始批量上传 {len(file_paths)} 个文件...")
            results = uploader.batch_upload(file_paths)
            
            # 统计结果
            success_count = sum(1 for r in results if r['success'])
            fail_count = len(results) - success_count
            
            print(f"\n📊 上传完成统计:")
            print(f"✅ 成功: {success_count} 个")
            print(f"❌ 失败: {fail_count} 个")
            
            # 显示失败的文件
            if fail_count > 0:
                print("\n失败的文件:")
                for result in results:
                    if not result['success']:
                        print(f"  - {result['file_path']}: {result['error']}")
            
    except Exception as e:
        print(f"\n❌ 错误: {str(e)}")
        sys.exit(1)


# 使用示例
if __name__ == "__main__":
    main()


# ============== 编程接口使用示例 ==============

def example_usage():
    """编程接口使用示例"""
    
    # 创建上传器实例
    uploader = FileUploader("http://clusters.luo980.com:13012")
    
    # 示例1: 上传单个文件
    try:
        result = uploader.upload_file("example.txt")
        if result['code'] == 200:
            print(f"文件上传成功: {result['data']}")
        else:
            print(f"上传失败: {result['message']}")
    except Exception as e:
        print(f"上传出错: {e}")
    
    # 示例2: 上传文件并指定自定义文件名
    try:
        result = uploader.upload_file("local_file.jpg", "remote_image.jpg")
        print(f"上传结果: {result}")
    except Exception as e:
        print(f"上传出错: {e}")
    
    # 示例3: 批量上传文件
    file_list = ["file1.txt", "file2.jpg", "file3.pdf"]
    try:
        results = uploader.batch_upload(file_list)
        for result in results:
            if result['success']:
                print(f"✅ {result['file_path']} 上传成功")
            else:
                print(f"❌ {result['file_path']} 上传失败: {result['error']}")
    except Exception as e:
        print(f"批量上传出错: {e}")


# 取消注释下面的行来运行示例
# example_usage()
