import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os

def draw_numbered_circle(number, output_path="numbered_circle.png", width=512, height=512, font_scale=0.3):
    """
    绘制一个白底红色圆圈，圆圈上有白色数字的图片
    
    参数:
    number: 要显示的数字（两位数）
    output_path: 输出图片的路径
    width: 图片宽度
    height: 图片高度
    font_scale: 字体大小缩放因子（相对于半径的比例，默认0.3）
    """
    # 创建白色背景的图片
    image = Image.new('RGB', (width, height), 'white')
    draw = ImageDraw.Draw(image)
    
    # 计算圆圈的位置和大小（与图片边缘相切）
    center_x = width // 2
    center_y = height // 2
    radius = min(width, height) // 2  # 圆圈半径等于图片宽高的一半，与边缘相切
    
    # 绘制红色圆圈
    circle_bbox = [center_x - radius, center_y - radius, 
                   center_x + radius, center_y + radius]
    draw.ellipse(circle_bbox, fill='red', outline='red')
    
    # 设置字体（尝试使用系统字体，如果没有则使用默认字体）
    try:
        # 根据font_scale参数调整字体大小
        font_size = int(radius * font_scale)
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
    except:
        try:
            # 备选字体
            font = ImageFont.truetype("arial.ttf", int(radius * font_scale))
        except:
            # 使用默认字体
            font = ImageFont.load_default()
    
    # 准备要显示的数字文本
    text = f"{number:02d}"  # 确保是两位数格式
    
    # 获取文本的边界框来计算居中位置
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    
    # 计算文字的居中位置
    text_x = center_x - text_width // 2
    text_y = center_y - text_height // 1.35
    
    # 在圆圈上绘制白色数字
    draw.text((text_x, text_y), text, fill='white', font=font)
    
    # 保存图片
    image.save(output_path)
    print(f"图片已保存到: {output_path}")
    return image

if __name__ == "__main__":
    # 创建输出目录
    os.makedirs("numpic", exist_ok=True)

    # 你也可以绘制其他数字
    for i in range(1, 11):
        draw_numbered_circle(i, f"num_pic/{i:02d}.png", font_scale=1.3)
        
    print("完成！已生成多个示例图片。")