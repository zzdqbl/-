#!/usr/bin/env python3
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class StartButtonNode(Node):
    def __init__(self):
        super().__init__('start_button_node')
        self.publisher_ = self.create_publisher(String, 'start_topic', 10)
        # <--- 新增：创建 reset 话题发布者 --->
        self.reset_publisher_ = self.create_publisher(String, 'reset_topic', 10) 

    def publish_start(self):
        msg = String()
        msg.data = "start"
        self.publisher_.publish(msg)
        self.get_logger().info("🖱️ 已发布 'start' 消息")
        global start_button
        start_button.config(text="点击开始比赛", state="normal", bg="#4CAF50", fg="white")

    # <--- 新增：定义 reset 发布函数 --->
    def publish_reset(self):
        msg = String()
        msg.data = "reset"  # 假设主逻辑监听 "reset" 消息
        self.reset_publisher_.publish(msg)
        self.get_logger().info("🔄 已发布 'reset' 消息")
        
        # 可选：重置按钮状态，使其恢复可点击
        global reset_button
        # 这里可以改变 Reset 按钮的外观，或者保持不变
        # reset_button.config(text="已重置", state="disabled", bg="#888888")

def main(args=None):
    rclpy.init(args=args)
    node = StartButtonNode()

    root = tk.Tk()
    root.title("智能车比赛 - 启动终端")
    root.geometry("500x300")
    root.resizable(False, False)

    # 定义按钮变量
    global start_button
    global reset_button # <--- 新增：定义 reset 按钮变量 --->

    # Start 按钮
    start_button = tk.Button(
        root, 
        text="点击开始比赛", 
        command=node.publish_start,
        width=10,
        height=2,
        font=("Microsoft YaHei", 16, "bold"),
        bg="#4CAF50",
        fg="white",
        activebackground="#45a049",
        relief="raised",
        bd=5
    )
    start_button.pack(side="left", expand=True, fill="both", padx=20, pady=40)

    # <--- 新增：Reset 按钮 --->
    # 注意：这个按钮需要放在 Start 按钮下面
    reset_button = tk.Button(
        root,
        text="清空重开",       # 按钮文字
        command=node.publish_reset, # 绑定新函数
        width=10,
        height=2,
        font=("Microsoft YaHei", 16, "bold"),
        bg="#FF9800",         # 橙色背景，区分功能
        fg="white",
        activebackground="#FB8C00",
        relief="raised",
        bd=5
    )
    reset_button.pack(side="right", expand=True, fill="both", padx=20, pady=40)

    def on_closing():
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    def spin_ros():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()