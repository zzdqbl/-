#!/usr/bin/env python3
import rclpy
import math
import json
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String

from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

from rcl_interfaces.msg import Log


class PipelineNavigator(Node):
    def __init__(self):
        super().__init__('pipeline_navigator')

        self.declare_parameter('waypoints', '')
        wp_param = self.get_parameter('waypoints').value
        
        self.full_waypoints = []
        if isinstance(wp_param, str) and wp_param.strip():
            self.full_waypoints = json.loads(wp_param)
        
        if len(self.full_waypoints) < 2:
            self.get_logger().error("路点数量不足！")
            return

        # ===================== 状态机 =====================
        self.start_signal_received = False
        self.is_nav_running = False
        
        self.current_pass_idx = 0
        self.next_send_idx = 3
        self.total_waypoints = len(self.full_waypoints)
        self.waypoint_arrive_dist = 0.50

        # ===================== 失败安全保护（🔥 关键）=====================
        self.nav_failure_count = 0
        self.max_failure_retries = 1    # 最多试2次
        self.enable_recovery = True     # 恢复开关，可关闭
        self.sent_midpoint_boost = False  # 加在这里
        self._failure_recovery_cooldown = False

        self.MAX_STEP = 2              # 最多生成3个回退点
        self.STEP_DISTANCE = 0.4      # 每一步后退 0.25 米

        self.rosout_callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10,
            callback_group=self.rosout_callback_group
        )
        
        # ===================== Nav2 =====================
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses",
            callback_group=self.callback_group
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_frame = "map"
        self.base_frame = "base_link"

        # 监听 & 定时器（必须加回调组）
        self.create_subscription(String, 'start_topic', self.start_callback, 10)
        self.create_subscription(String, 'reset_topic', self.reset_callback, 10)
        self.create_timer(0.2, self.waypoint_check_timer, callback_group=self.callback_group)

        self.get_logger().info("✅ 安全版流水线导航启动")

    # ===================== 启动 =====================
    def start_callback(self, msg):
        if msg.data == "start" and not self.start_signal_received:
            self.get_logger().info("🚀 启动 [0,1]")
            self.start_signal_received = True
            self.is_nav_running = True
            self.current_pass_idx = 0
            self.next_send_idx = 3
            self.send_goal([0, 1, 2])

     # 🔥 ===================== 新增：重置回调 =====================
    def reset_callback(self, msg):
        if msg.data == "reset":
            self.get_logger().warn("🔄 收到重置命令！全部状态清空，回到初始状态！")
            
            # 停止导航
            self.is_nav_running = False
            self.start_signal_received = False
            
            # 重置所有状态
            self.current_pass_idx = 0
            self.next_send_idx = 3
            self.nav_failure_count = 0
            self.sent_midpoint_boost = False
            self._failure_recovery_cooldown = False

            self.get_logger().info("✅ 重置完成！可以再次点击开始")

    # ===================== 正常流水线 =====================
    def waypoint_check_timer(self):
        if not self.is_nav_running:
            return

        x, y = self.get_robot_pose()
        if x is None:
            return

        if self.current_pass_idx >= self.total_waypoints:
            self.is_nav_running = False
            self.get_logger().info("🎉 全部完成")
            return

        target_idx = self.current_pass_idx
        wx, wy = self.full_waypoints[target_idx][0], self.full_waypoints[target_idx][1]
        dist = math.hypot(x - wx, y - wy)

        check_idxs = [self.current_pass_idx,
              self.current_pass_idx + 1,
              self.current_pass_idx + 2]
        check_idxs = [i for i in check_idxs if i < self.total_waypoints]

        # 找最远到达的点
        reached = self.current_pass_idx
        for idx in check_idxs:
            wx_, wy_ = self.full_waypoints[idx][0], self.full_waypoints[idx][1]
            d_ = math.hypot(x - wx_, y - wy_)
            if d_ < self.waypoint_arrive_dist:
                reached = idx

        # 如果跳到更远的点，直接更新
        if reached > self.current_pass_idx:
            self.get_logger().info(f"✅ 跳点到达 [{reached}]")
            self.current_pass_idx = reached
            dist = 0  # 触发下面的到达逻辑
            return  
         # 🔥 中点强制补发（你要的功能）
        # =====================
        if not self.sent_midpoint_boost and self.current_pass_idx + 1 < self.total_waypoints:
            x_idx = self.current_pass_idx
            x1_idx = self.current_pass_idx + 1

            x2 = self.full_waypoints[x_idx][0]
            y2 = self.full_waypoints[x_idx][1]
            x1 = self.full_waypoints[x1_idx][0]
            y1 = self.full_waypoints[x1_idx][1]

            mid_x = (x2 + x1) / 2
            mid_y = (y2 + y1) / 2

            robot_to_mid = math.hypot(x - mid_x, y - mid_y)

            if robot_to_mid < 0.35:  # 走过中点
                if self.next_send_idx < self.total_waypoints:
                    self.get_logger().warn("⚡ 过中点，强制补发下一组点")
                    send_list = [self.next_send_idx - 2, self.next_send_idx- 1,self.next_send_idx ]
                    self.send_goal(send_list)
                    self.next_send_idx += 1
                    self.sent_midpoint_boost = True

                    self.current_pass_idx += 1

        # ====================================================================

        if dist < self.waypoint_arrive_dist:
            self.get_logger().info(f"✅ 到达 [{self.current_pass_idx}]")
            self.current_pass_idx += 1
            self.nav_failure_count = 0  # 成功就清空错误
            self.sent_midpoint_boost = False  # 🔥 加这一句

            if self.next_send_idx < self.current_pass_idx + 3:
                self.next_send_idx = self.current_pass_idx + 2

            # 追加下一组
            if self.next_send_idx < self.total_waypoints:
                send_list = [self.next_send_idx - 2, self.next_send_idx-1 ,self.next_send_idx]
                self.send_goal(send_list)
                self.next_send_idx += 1

            return

        #  # ===================== 🔥 你要的：最保险距离判断（核心） =====================
        # # 机器人 到 上一个点 的距离 > 上一个点到当前点的路段长度 → 发下一组
        # triggered = False
        # if self.current_pass_idx >= 1 and self.next_send_idx < self.total_waypoints:
        #     prev_idx = self.current_pass_idx - 1    # 上一个点
        #     curr_idx = self.current_pass_idx        # 当前点

        #     # 上一个点坐标
        #     px, py = self.full_waypoints[prev_idx][0], self.full_waypoints[prev_idx][1]
        #     # 当前点坐标
        #     cx, cy = self.full_waypoints[curr_idx][0], self.full_waypoints[curr_idx][1]

        #     # 机器人到上一个点的距离
        #     robot_prev_dist = math.hypot(x - px, y - py)
        #     # 路段长度：上一个点 → 当前点
        #     segment_dist = math.hypot(cx - px, cy - py)

        #     # ====================== 核心判断 ======================
        #     if robot_prev_dist > segment_dist*0.95 :  # 1.0m/s 最稳阈值
        #         if not self.sent_midpoint_boost:
        #             self.get_logger().warn(f"⚡ 保险触发：走过路段长度，下发下一组")
        #             send_list = [self.next_send_idx - 2, self.next_send_idx-1, self.next_send_idx]
        #             self.send_goal(send_list)
        #             self.next_send_idx += 1
        #             self.sent_midpoint_boost = True
        #             self.current_pass_idx += 1
        #             triggered = True
        #     if triggered:
        #         return
        # # =====================
       
        # 🔥 3. 新增：超距防失误（和中点平级，谁先触发谁生效）
        # if not self.sent_midpoint_boost and self.current_pass_idx + 1 < self.total_waypoints:
        #     # 当前点（1）和下一个点（2）的坐标
        #     x1 = self.full_waypoints[self.current_pass_idx][0]
        #     y1 = self.full_waypoints[self.current_pass_idx][1]
        #     x2 = self.full_waypoints[self.current_pass_idx + 1][0]
        #     y2 = self.full_waypoints[self.current_pass_idx + 1][1]

        #     # 计算 1→2 的直线距离
        #     dist_1_to_2 = math.hypot(x2 - x1, y2 - y1)
        #     # 计算 机器人当前位置 → 点1 的距离（判断是否没到点1）
        #     dist_robot_to_1 = math.hypot(x - x1, y - y1)

        #     # 核心判断：没到点1 + 走的距离超过（1→2距离 + 0.5m）→ 强制补发
        #     if dist_robot_to_1 >= self.waypoint_arrive_dist and dist_robot_to_1 > (dist_1_to_2 + 0.8):
        #         if self.next_send_idx < self.total_waypoints:
        #             self.get_logger().warn(f"⚠️  超距防失误：已走{dist_robot_to_1:.2f}m（超1→2距离+0.8m），强制补发下一组")
        #             send_list = [self.next_send_idx - 2, self.next_send_idx-1,self.next_send_idx]
        #             self.send_goal(send_list)
        #             self.next_send_idx += 1
        #             self.sent_midpoint_boost = True

        #             self.current_pass_idx += 1

    # ===================== 发送路点（无锁，纯追加）=====================
    def send_goal(self, indices):
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return
        goal = NavigateThroughPoses.Goal()
        for idx in indices:
            if idx < len(self.full_waypoints):
                goal.poses.append(self.create_pose(self.full_waypoints[idx]))
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_cb)
        self.get_logger().info(f"📤 发送：{indices}")

    # ===================== 回调 =====================
    def goal_cb(self, future):
        try:
            handle = future.result()
            if handle:
                handle.get_result_async().add_done_callback(self.result_cb)
        except:
            pass

    def rosout_callback(self, msg):
        if not self.is_nav_running:
            return
        # if self._failure_recovery_cooldown:
        #     return
        if msg.level < 4:
            return
        
        
        error_keywords = [
            "failed to create plan",
            "Goal failed",
            "Planning algorithm failed",
            "Starting point in lethal space",
            
        ]

        for kw in error_keywords:
            if kw in msg.msg:
                # self.get_logger().warn("⚠️  连续失败2次 → 发送虚拟点")
                # self._failure_recovery_cooldown = True
                # self.try_recovery()
                # timer = self.create_timer(0.3, lambda: setattr(self, '_failure_recovery_cooldown', False))
                # timer.cancel()
                # 读到一次就重发一次################
                self.get_logger().warn("⚠️ 检测到导航失败，立即重发当前路点组")
                self.try_recovery()  # 直接重发
                return
   # ===================== 【安全恢复】最多试2次，不无限循环 =====================
    def try_recovery(self):
        if not self.enable_recovery:
            return
        idx = self.current_pass_idx
        if idx >= self.total_waypoints:
            return

        # 清除代价地图（防止障碍占用）
        try:
            from nav2_msgs.srv import ClearEntireCostmap
            if not hasattr(self, 'clear_costmap_client'):
                self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entire_costmap')
            if self.clear_costmap_client.wait_for_service(timeout_sec=0.1):
                self.clear_costmap_client.call_async(ClearEntireCostmap.Request())
        except:
            pass

        # 永远重发：当前点开始的 3 个点
        send_list = []
        for i in range(3):
            if idx + i < self.total_waypoints:
                send_list.append(idx + i)

        if send_list:
            self.get_logger().info(f"🔁 重发路点组：{send_list}")
            self.send_goal(send_list)
######################################
        # idx = self.current_pass_idx
        # if idx >= self.total_waypoints:
        #     self._failure_recovery_cooldown = False
        #     return

        # self.nav_failure_count += 1

        # is_last_point = (self.current_pass_idx == self.total_waypoints - 1)
        # ###############################
        # if is_last_point:
        #     self.get_logger().warn(f"⚠️ 最后一个点，继续尝试中... 失败次数: {self.nav_failure_count}")
        # else:
        #     self.get_logger().warn(f"⚠️ 重试当前航点组，失败次数: {self.nav_failure_count}")

        # # 🔥 永远只发：当前点开始的 3 个点（一模一样）
        # try:
        #     from nav2_msgs.srv import ClearEntireCostmap
        #     if not hasattr(self, 'clear_costmap_client'):
        #         self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entire_costmap')
        #     if self.clear_costmap_client.wait_for_service(timeout_sec=0.1):
        #         self.clear_costmap_client.call_async(ClearEntireCostmap.Request())
        # except:
        #     pass
        # send_list = []
        # for i in range(3):
        #     if idx + i < self.total_waypoints:
        #         send_list.append(idx + i)

        # if send_list:
        #     self.send_goal(send_list)

        # self._failure_recovery_cooldown = False
        ###############################
        # 超过2次失败直接跳过
        # if self.nav_failure_count >= self.max_failure_retries:
        #     self.get_logger().error("❌ 多次恢复失败，跳过该点！")
        #     self.current_pass_idx += 1
        #     self.nav_failure_count = 0
        #     self.sent_midpoint_boost = False
        #     self._failure_recovery_cooldown = False
        #     self.next_send_idx = self.current_pass_idx + 2
        #     # 🔥 【正确逻辑】发送：当前新点 + 下两个点
        #     curr = self.current_pass_idx
        #     send_list = []
        #     for i in range(3):
        #         if curr + i < self.total_waypoints:
        #             send_list.append(curr + i)

        #     if send_list:
        #         self.send_goal(send_list)
        #         self.next_send_idx = curr + 2  # 同步索引

        #     # 【关键：解锁！】
            
        #     return
        # self._failure_recovery_cooldown = False
        #########################
        # # 获取机器人当前位置
        # rx, ry = self.get_robot_pose()
        # if rx is None:
        #     self._failure_recovery_cooldown = False
        #     return

        # # 当前目标点
        # tx, ty = self.full_waypoints[idx][0], self.full_waypoints[idx][1]

        # # 已经接近 -> 算到达
        # if math.hypot(rx - tx, ry - ty) < 0.4:
        #     self.get_logger().info("✅ 已接近，视为到达")
        #     self.current_pass_idx += 1
        #     self.nav_failure_count = 0
        #     self.sent_midpoint_boost = False
        #     self._failure_recovery_cooldown = False
        #     return

        # # 计算机器人到目标点距离
        # d = math.hypot(rx - tx, ry - ty)
        # if d < 0.3:
        #     self._failure_recovery_cooldown = False
        #     return

        # # 🔥 生成回退点（从目标点往机器人方向退）
        # backoff_points = []
        # for step in range(1, self.MAX_STEP + 1):
        #     dist = step * self.STEP_DISTANCE
        #     if dist >= d:
        #         break
        #     scale = dist / d
        #     x = tx + (rx - tx) * scale
        #     y = ty + (ry - ty) * scale
        #     backoff_points.append((x, y))

        # # 发送第一个回退点
        # if backoff_points:
        #     vx, vy = backoff_points[0]
        #     self.send_virtual_goal(vx, vy, self.full_waypoints[idx][2], idx)
        #     self.get_logger().info(f"🚑 发回退虚拟点 → [{idx}] ({vx:.2f}, {vy:.2f})")

    def send_virtual_goal(self, x, y, yaw_deg, target_idx):
        goal = NavigateThroughPoses.Goal()
        p1 = PoseStamped()
        p1.header.frame_id = "map"
        p1.pose.position.x = x
        p1.pose.position.y = y
        p1.pose.orientation = self.yaw_to_quat(math.radians(yaw_deg))
        goal.poses.append(p1)

        next_idx = target_idx + 1
        if next_idx < len(self.full_waypoints):
            goal.poses.append(self.create_pose(self.full_waypoints[next_idx]))

        self.nav_client.send_goal_async(goal)

    def get_closest_free_point(self, x, y, max_radius=0.6, step=0.1):
        directions = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        for r in [step*i for i in range(1, int(max_radius/step)+1)]:
            for dx, dy in directions:
                sx = x + dx*r
                sy = y + dy*r
                if self.is_cost_free(sx, sy):
                    return sx, sy
        return x, y

    def is_cost_free(self, x, y):
        try:
            # 这里简化：不依赖代价地图服务，避免启动报错
            # 真实环境可替换成 costmap 检查
            return True
        except:
            return True


    # ===================== 工具 =====================
    def create_pose(self, pt):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation = self.yaw_to_quat(math.radians(pt[2]))
        return pose

    def yaw_to_quat(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw/2)
        q.z = math.sin(yaw/2)
        return q

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = PipelineNavigator()
    exe = MultiThreadedExecutor(4)
    exe.add_node(node)
    exe.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()