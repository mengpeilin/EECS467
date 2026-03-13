/**
 * 位置接收节点 - C++ 版本
 * 
 * 用于接收其他机器人的定位信息
 * 
 * 编译: 
 *   cd /home/mbot/mbot_ros_labs
 *   colcon build --packages-select mbot_localization
 * 
 * 运行:
 *   ros2 run mbot_localization localization_pose_listener
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

class LocalizedPoseListener : public rclcpp::Node
{
public:
    LocalizedPoseListener() : Node("localization_pose_listener")
    {
        RCLCPP_INFO(get_logger(), "=== 位置监听节点启动 ===");
        
        // 订阅Robot #1的位置
        subscription_robot1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localized_pose_robot1",
            10,
            std::bind(&LocalizedPoseListener::pose_callback_robot1, this, std::placeholders::_1));
        
        // 订阅Robot #2的位置
        subscription_robot2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localized_pose_robot2",
            10,
            std::bind(&LocalizedPoseListener::pose_callback_robot2, this, std::placeholders::_1));
        
        // 订阅状态消息
        status_sub_robot1_ = this->create_subscription<std_msgs::msg::String>(
            "/localization_status_robot1",
            10,
            std::bind(&LocalizedPoseListener::status_callback_robot1, this, std::placeholders::_1));
        
        status_sub_robot2_ = this->create_subscription<std_msgs::msg::String>(
            "/localization_status_robot2",
            10,
            std::bind(&LocalizedPoseListener::status_callback_robot2, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "订阅话题: /localized_pose_robot1, /localized_pose_robot2");
        RCLCPP_INFO(get_logger(), "等待机器人定位成功...");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_robot1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_robot2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_robot1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_robot2_;
    
    void pose_callback_robot1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        handle_pose_message("robot1", msg);
    }
    
    void pose_callback_robot2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        handle_pose_message("robot2", msg);
    }
    
    void handle_pose_message(const std::string& robot_name, 
                            const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        const auto& pose = msg->pose.pose;
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;
        
        // 四元数转欧拉角
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;
        
        double yaw = std::atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz));
        
        RCLCPP_INFO(get_logger(),
            "[%s 已定位] 位置: (%.3f, %.3f, %.3f), 方向: %.1f°, 坐标系: %s",
            robot_name.c_str(), x, y, z, 
            yaw * 180.0 / M_PI, msg->header.frame_id.c_str());
        
        // 执行任务相关逻辑
        execute_mission_logic(robot_name, x, y, yaw);
    }
    
    void execute_mission_logic(const std::string& robot_name, double x, double y, double yaw)
    {
        if (robot_name == "robot1") {
            RCLCPP_INFO(get_logger(),
                "\n🤖 Robot #1 已定位，准备执行任务:\n"
                "  1. 驾驶到指定位置 (%.2f, %.2f)\n"
                "  2. 打开Robot #2的牢房\n"
                "  3. 返回起点附近",
                x, y);
        } else if (robot_name == "robot2") {
            RCLCPP_INFO(get_logger(),
                "\n🤖 Robot #2 已定位，准备执行任务:\n"
                "  1. 当前位置: (%.2f, %.2f)\n"
                "  2. 规划回到起点的路径\n"
                "  3. 避免与Robot #1碰撞",
                x, y);
        }
    }
    
    void status_callback_robot1(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "[Robot1 状态] %s", msg->data.c_str());
    }
    
    void status_callback_robot2(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "[Robot2 状态] %s", msg->data.c_str());
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizedPoseListener>());
    rclcpp::shutdown();
    return 0;
}
