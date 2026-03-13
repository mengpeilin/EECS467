#!/usr/bin/env python3
"""
位置接收节点 - 用于接收其他机器人的定位信息

场景：
- Robot #1 定位后，向 `/localized_pose_robot1` 发布位置
- Robot #2 定位后，向 `/localized_pose_robot2` 发布位置
- 其他节点可订阅这些话题获取机器人位置，进行后续导航

使用方式：
    ros2 run mbot_localization pose_listener_node.py

或通过Python直接运行：
    python3 pose_listener_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import math

class LocalizedPoseListener(Node):
    """监听并处理机器人的定位位置"""
    
    def __init__(self):
        super().__init__('localized_pose_listener')
        
        self.get_logger().info("=== 位置监听节点启动 ===")
        
        # 存储收到的位置信息
        self.robots_poses = {}
        
        # 订阅Robot #1的位置
        self.subscription_robot1 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localized_pose_robot1',
            self.pose_callback_robot1,
            10)
        
        # 订阅Robot #2的位置
        self.subscription_robot2 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localized_pose_robot2',
            self.pose_callback_robot2,
            10)
        
        # 订阅状态消息
        self.status_sub_robot1 = self.create_subscription(
            String,
            '/localization_status_robot1',
            self.status_callback_robot1,
            10)
        
        self.status_sub_robot2 = self.create_subscription(
            String,
            '/localization_status_robot2',
            self.status_callback_robot2,
            10)
        
        self.get_logger().info("订阅话题: /localized_pose_robot1, /localized_pose_robot2")
        self.get_logger().info("等待机器人定位成功...")
    
    def pose_callback_robot1(self, msg):
        """处理Robot #1的位置消息"""
        self._handle_pose_message('robot1', msg)
    
    def pose_callback_robot2(self, msg):
        """处理Robot #2的位置消息"""
        self._handle_pose_message('robot2', msg)
    
    def _handle_pose_message(self, robot_name, msg):
        """通用的位置消息处理函数"""
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # 存储位置信息
        self.robots_poses[robot_name] = {
            'x': x,
            'y': y,
            'z': z,
            'frame': msg.header.frame_id,
            'timestamp': msg.header.stamp
        }
        
        # 提取方向（四元数转欧拉角）
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
        
        self.get_logger().info(
            f"[{robot_name.upper()} 已定位]\n"
            f"  位置: ({x:.3f}, {y:.3f}, {z:.3f})\n"
            f"  方向: {math.degrees(yaw):.1f}°\n"
            f"  坐标系: {msg.header.frame_id}\n"
            f"  时间: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )
        
        # 在这里可以实现后续逻辑
        self._execute_mission_logic(robot_name, x, y, yaw)
    
    def status_callback_robot1(self, msg):
        """处理Robot #1的状态消息"""
        self.get_logger().info(f"[Robot1 状态] {msg.data}")
    
    def status_callback_robot2(self, msg):
        """处理Robot #2的状态消息"""
        self.get_logger().info(f"[Robot2 状态] {msg.data}")
    
    def _execute_mission_logic(self, robot_name, x, y, yaw):
        """根据机器人名称执行不同的任务逻辑"""
        
        if robot_name == 'robot1':
            self.get_logger().info(
                f"\n🤖 Robot #1 已定位，准备执行任务:\n"
                f"  1. 驾驶到指定位置 ({x:.2f}, {y:.2f})\n"
                f"  2. 打开Robot #2的牢房\n"
                f"  3. 返回起点附近\n"
            )
            # 这里可以调用导航服务或发送导航指令
            # 例如：self.navigate_to_position(x, y, yaw)
            
        elif robot_name == 'robot2':
            self.get_logger().info(
                f"\n🤖 Robot #2 已定位，准备执行任务:\n"
                f"  1. 当前位置: ({x:.2f}, {y:.2f})\n"
                f"  2. 规划回到起点的路径\n"
                f"  3. 避免与Robot #1碰撞\n"
            )
            # 这里可以调用导航服务
            # 例如：self.navigate_to_origin()
    
    def get_robot_pose(self, robot_name):
        """获取指定机器人的位置信息"""
        return self.robots_poses.get(robot_name, None)
    
    def calculate_distance(self, robot1, robot2):
        """计算两个机器人之间的距离"""
        if robot1 not in self.robots_poses or robot2 not in self.robots_poses:
            return None
        
        pose1 = self.robots_poses[robot1]
        pose2 = self.robots_poses[robot2]
        
        dx = pose1['x'] - pose2['x']
        dy = pose1['y'] - pose2['y']
        
        return math.sqrt(dx*dx + dy*dy)
    
    def print_all_robots_status(self):
        """打印所有机器人的状态"""
        self.get_logger().info("\n=== 所有机器人状态 ===")
        for robot_name, pose in self.robots_poses.items():
            self.get_logger().info(
                f"{robot_name}: ({pose['x']:.2f}, {pose['y']:.2f})"
            )
        
        # 计算机器人之间的距离
        if len(self.robots_poses) >= 2:
            distance = self.calculate_distance('robot1', 'robot2')
            if distance is not None:
                self.get_logger().info(f"Robot1 与 Robot2 距离: {distance:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    listener = LocalizedPoseListener()
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
