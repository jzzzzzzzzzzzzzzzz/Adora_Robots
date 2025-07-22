# keyboard_teleop/keyboard_teleop/main.py (最终安全调试版)
import sys
import json
import time

try:
    import readchar
except ImportError:
    print("错误: 未找到 'readchar' 库。", file=sys.stderr)
    print("请先运行: pip install readchar", file=sys.stderr)
    sys.exit(1)

from dora import Node

def main():
    print("--- 键盘遥控客户端 (最终安全调试模式) ---")
    print("警告: 速度已限制在极低水平，用于安全调试！")
    print("用法: w/a/s/d/x 移动 | 'q' 退出")

    try:
        node = Node("keyboard_teleop")
    except Exception as e:
        print(f"致命错误: 无法连接到 Dora 网络。请确保 'dora start' 正在运行。", file=sys.stderr)
        sys.exit(1)

    # --- 绝对安全的速度设置 ---
    # 速度单位: 米/秒 和 弧度/秒
    DEBUG_LINEAR_SPEED = 0.10  # 极其缓慢的前进速度 (5 厘米/秒)
    DEBUG_ANGULAR_SPEED = 0.2  # 极其缓慢的转向速度 (约 6 度/秒)
    
    try:
        while True:
            key = readchar.readkey()

            # --- 每次循环都显式重置速度，确保安全 ---
            linear_x, angular_z = 0.0, 0.0

            # --- 明确的、无歧义的逻辑判断 ---
            if key == 'w' or key == '\x1b[A': # 前
                linear_x = DEBUG_LINEAR_SPEED
            elif key == 'x' or key == '\x1b[B': # 后
                linear_x = -DEBUG_LINEAR_SPEED
            elif key == 'a' or key == '\x1b[D': # 左转
                angular_z = DEBUG_ANGULAR_SPEED
            elif key == 'd' or key == '\x1b[C': # 右转
                angular_z = -DEBUG_ANGULAR_SPEED
            elif key == 's': # 显式停止
                pass # linear_x 和 angular_z 已经在本轮循环开始时被重置为 0
            
            # --- 移除所有速度切换档位，只使用一个安全速度 ---
            
            if key == 'q':
                # 确保在退出前发送明确的停止指令
                stop_msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}
                node.send_output("CmdVelTwist", json.dumps(stop_msg).encode('utf-8'))
                print("正在发送停止指令并退出...")
                break

            # 构建并发送消息
            msg = {"linear":{"x": linear_x, "y": 0.0, "z": 0.0}, "angular":{"x": 0.0, "y": 0.0, "z": angular_z}}
            print(f"发送安全指令 -> {msg}")
            node.send_output("CmdVelTwist", json.dumps(msg).encode('utf-8'))

    except KeyboardInterrupt:
        print("\n检测到中断，正在安全退出...")
    finally:
        # 确保在任何情况下退出时都发送停止指令
        stop_msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}
        node.send_output("CmdVelTwist", json.dumps(stop_msg).encode('utf-8'))
        print("遥控客户端已关闭。")


if __name__ == '__main__':
    main()