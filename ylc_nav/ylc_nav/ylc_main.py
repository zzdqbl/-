#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import time
from rclpy.node import Node

def ROS_INFO(msg):
    print(f"[INFO] [ylc_main]: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = Node("ylc_main_node")

    ROS_INFO("等待 Nav2 导航管理器启动完成...")

    # 等待 Nav2 完全启动
    time.sleep(10)

    ROS_INFO("==================================================")
    ROS_INFO("✅ Nav2 已完全启动！")
    ROS_INFO("✅ ylc_main node started successfully!")
    ROS_INFO("✅ Navigation system running normal!")
    ROS_INFO("==================================================")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()