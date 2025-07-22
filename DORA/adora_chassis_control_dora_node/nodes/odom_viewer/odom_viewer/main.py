# odom_viewer/odom_viewer/main.py (最终修正版)

import sys
import json
from dora import Node

def main():
    print("--- 机器人状态查看器 (动态节点模式) ---")
    
    try:
        node = Node("odom_viewer")
    except Exception as e:
        print(f"致命错误: 无法连接到 Dora 网络并认领节点 'odom_viewer'。", file=sys.stderr)
        print(f"请确保 'dora start' 正在另一个终端中运行。", file=sys.stderr)
        sys.exit(1)

    print("正在等待来自数据流的数据...")

    try:
        for event in node:
            if event["type"] == "INPUT":
                try:
                    # --- 关键修复：处理收到的整数列表 ---
                    
                    # 1. 原始数据是一个 Arrow Array
                    raw_value = event["value"]
                    
                    # 2. 将 Arrow Array 转换为 Python 列表
                    byte_list = raw_value.to_pylist()
                    
                    # 3. 将整数列表 (ASCII 码) 转换为字节数组 (bytes)
                    byte_array = bytes(byte_list)
                    
                    # 4. 将字节数组解码为 UTF-8 字符串
                    json_string = byte_array.decode('utf-8')
                    
                    # 5. 现在，我们可以安全地解析这个正确的 JSON 字符串了
                    data = json.loads(json_string)
                    
                    # --- 后续的显示逻辑保持不变 ---
                    pos = data['pose']['position']
                    twist = data['twist']['linear']
                    angular = data['twist']['angular']
                    
                    print(f"\r位置: X={pos['x']:.2f}, Y={pos['y']:.2f} | 速度: X={twist['x']:.2f} | 角速度: Z={angular['z']:.2f}   ", end="", flush=True)

                except Exception as e:
                    # 提供更详细的错误信息，帮助调试
                    print(f"\n[错误] 处理输入时发生错误: {e}", file=sys.stderr)
                    # 打印出有问题的原始数据，以便分析
                    if 'byte_list' in locals():
                         print(f"  - 原始字节列表 (前50字节): {byte_list[:50]}", file=sys.stderr)
                    if 'json_string' in locals():
                         print(f"  - 重建的字符串 (前100字符): {json_string[:100]}", file=sys.stderr)
            
            elif event["type"] == "STOP":
                break
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        print("\n状态查看器已关闭。")

if __name__ == '__main__':
    main()