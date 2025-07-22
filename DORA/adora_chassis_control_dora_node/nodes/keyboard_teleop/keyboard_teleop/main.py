# /root/workspace/Adora_Robots/DORA/adora_chassis_control_dora_node/nodes/keyboard_teleop/keyboard_teleop/main.py (最终匹配版)

import sys
import json
import time
from datetime import datetime

try:
    import readchar
except ImportError:
    print("错误: 未找到 'readchar' 库。", file=sys.stderr)
    print("请先运行: pip install readchar", file=sys.stderr)
    sys.exit(1)

from dora import Node

def main():
    print("--- 键盘遥控客户端 (Adora 适配版) ---")
    print("用法: w/a/s/d/x 移动 | 'q' 退出")

    try:
        node = Node("keyboard_teleop")
    except Exception as e:
        print(f"致命错误: 无法连接到 Dora 网络。请确保 'dora start' 正在运行。", file=sys.stderr)
        sys.exit(1)

    # 安全速度设置
    LINEAR_SPEED = 0.1  # 基础前进速度 0.2 m/s
    ANGULAR_SPEED = 0.25 # 基础转向速度 0.25 rad/s
    
    # 用于消息序列号的计数器
    seq_counter = 0

    try:
        while True:
            key = readchar.readkey()

            linear_x, angular_z = 0.0, 0.0

            if key == 'w' or key == '\x1b[A': # 前
                linear_x = LINEAR_SPEED
            elif key == 'x' or key == '\x1b[B': # 后
                linear_x = -LINEAR_SPEED
            elif key == 'a' or key == '\x1b[D': # 左转
                angular_z = ANGULAR_SPEED
            elif key == 'd' or key == '\x1b[C': # 右转
                angular_z = -ANGULAR_SPEED
            elif key == 's': # 停止
                pass
            
            if key == 'q':
                # 确保在退出前发送明确的停止指令
                stop_msg_dict = build_message(0, 0, seq_counter)
                node.send_output("CmdVelTwist", json.dumps(stop_msg_dict).encode('utf-8'))
                print("正在发送停止指令并退出...")
                break
            
            seq_counter += 1
            # --- 关键修改：构建与 C++ 期望的完整消息结构 ---
            message_dict = build_message(linear_x, angular_z, seq_counter)

            # 发送完整的 JSON 消息
            json_string = json.dumps(message_dict, indent=4) # indent=4 与原始脚本行为一致
            print(f"发送指令 -> \n{json_string}")
            node.send_output("CmdVelTwist", json_string.encode('utf-8'))

    except KeyboardInterrupt:
        print("\n检测到中断，正在安全退出...")
    finally:
        print("遥控客户端已关闭。")

def build_message(linear_x, angular_z, seq):
    """一个辅助函数，用于构建符合 C++ 节点期望的完整 JSON 消息。"""
    timestamp = datetime.now().timestamp()
    message = {
        "header": {
            "frame_id": "keyboard",  # C++ 代码期望这个字段
            "seq": seq,              # C++ 代码期望这个字段
            "stamp": {
                "sec": int(timestamp),
                "nanosec": int((timestamp - int(timestamp)) * 1e9)
            }
        },
        "linear": {
            "x": linear_x,
            "y": 0.0,
            "z": 0.0
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": angular_z
        }
    }
    return message


if __name__ == '__main__':
    main()