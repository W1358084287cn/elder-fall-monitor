"""生成16x16和24x24中文字体位图数据"""
from PIL import Image, ImageDraw, ImageFont
import os

# 所有需要的中文字符
CHARS_16 = "血氧心率检测摔到秒后拨打求助电话触摸两次取消紧急已返回模式错误请重启设备配置连接热点访问地址离线纬经度卫星海拔定位中正在系统监测启动倒可救成功等待援信号发送稍候"
CHARS_24 = "一二三四五六七八九十"

# 查找Windows中文字体
font_paths = [
    "C:/Windows/Fonts/simhei.ttf",
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simsun.ttc",
    "C:/Windows/Fonts/Deng.ttf",
]

font_path = None
for fp in font_paths:
    if os.path.exists(fp):
        font_path = fp
        break

if not font_path:
    print("ERROR: No Chinese font found!")
    exit(1)

print(f"Using font: {font_path}")

def gen_char_bitmap(char, size, font_path):
    """生成单个字符的位图，返回C数组格式 (渲染2x大小再缩小提高质量)"""
    render_size = size * 2
    img = Image.new('L', (render_size, render_size), 0)
    draw = ImageDraw.Draw(img)
    
    # 尝试加载字体
    try:
        font = ImageFont.truetype(font_path, size * 2 - 4)
    except:
        font = ImageFont.load_default()
    
    # 计算字符边界框
    try:
        bbox = draw.textbbox((0, 0), char, font=font)
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        ox = bbox[0]
        oy = bbox[1]
    except AttributeError:
        w, h = draw.textsize(char, font=font)
        ox = 0
        oy = 0
    
    x = (render_size - w) // 2 - ox
    y = (render_size - h) // 2 - oy
    draw.text((x, y), char, font=font, fill=255)
    
    # 缩小到目标大小
    img = img.resize((size, size), Image.LANCZOS)
    
    # 转换为1-bit并导出为字节数组
    result = []
    for row in range(size):
        val = 0
        for col in range(size):
            if col > 0 and col % 8 == 0:
                result.append(val)
                val = 0
            if img.getpixel((col, row)) > 80:  # 阈值
                val |= (1 << (7 - (col % 8)))
        result.append(val)
    
    return result

def format_c_array(data, name, size):
    """格式化位图数据为C数组"""
    lines = []
    lines.append(f"// {size}x{size} font bitmap data")
    lines.append(f"static const uint8_t {name}[] PROGMEM = {{")
    
    hex_data = [f"0x{b:02X}" for b in data]
    for i in range(0, len(hex_data), 16):
        lines.append("    " + ", ".join(hex_data[i:i+16]) + ",")
    lines.append("};")
    return "\n".join(lines)

# 生成16x16字体
output = []
output.append("#pragma once")
output.append("// Auto-generated Chinese font bitmap data")
output.append("// Size: 16x16 pixels per character")
output.append("")
output.append("#include <Arduino.h>")
output.append("")
output.append(f"// Total characters: {len(CHARS_16)}")
output.append("static const char CN_FONT_CHARS[] PROGMEM = \"" + CHARS_16 + "\";")
output.append("")

for i, char in enumerate(CHARS_16):
    bitmap = gen_char_bitmap(char, 16, font_path)
    name = f"cn_font_{i:03d}"
    output.append(format_c_array(bitmap, name, 16))
    output.append("")

output.append("// Character bitmap lookup table (32 bytes per char)")
output.append("static const uint8_t* const CN_FONT_BITMAPS[] PROGMEM = {")
for i in range(len(CHARS_16)):
    output.append(f"    cn_font_{i:03d},")
output.append("};")
output.append("")
output.append(f"#define CN_FONT_COUNT {len(CHARS_16)}")
output.append(f"#define CN_FONT_SIZE 16")

# 写入文件
with open("src/chinese_font.h", "w", encoding="utf-8") as f:
    f.write("\n".join(output))

print(f"Generated {len(CHARS_16)} characters in src/chinese_font.h")
print(f"Model size: {len(CHARS_16) * 32} bytes")

# 打印字符映射表
print("\nCharacter map:")
for i, char in enumerate(CHARS_16):
    print(f"  [{i}] = '{char}'")