#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_nav/msg/pose2_d_array.hpp"
#include <rclcpp/qos.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DiffMotionController : public rclcpp::Node {
public:
    DiffMotionController() : Node("diff_motion_controller") {
        // TF setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        goal_subscriber_ = this->create_subscription<mbot_nav::msg::Pose2DArray>(
            "/waypoints", 10, std::bind(&DiffMotionController::goal_callback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Open log file
        log_file_.open("/home/mbot/mbot_ros_labs/src/mbot_nav/logs/pid_debug_log.csv");
            log_file_ << "time,distance_error,angle_error,lin_cmd,ang_cmd\n";
            log_file_.flush();

        timer_ = this->create_wall_timer(
            50ms, std::bind(&DiffMotionController::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "DiffMotionController initialized");
    }

    ~DiffMotionController() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    rclcpp::Subscription<mbot_nav::msg::Pose2DArray>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::ofstream log_file_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_theta_ = 0.0;
    bool goal_received_ = false;

    std::vector<geometry_msgs::msg::Pose2D> goal_queue_;

    double lin_error_sum_ = 0.0;
    double lin_error_last_ = 0.0;
    double ang_error_sum_ = 0.0;
    double ang_error_last_ = 0.0;

    void goal_callback(const mbot_nav::msg::Pose2DArray::SharedPtr msg) {
        // Clear old goals and fill queue with new ones
        goal_queue_.clear();
        for (const auto &pose : msg->poses) {
                goal_queue_.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "Received %zu goals", msg->poses.size());

        // Load the first goal
        if (!goal_queue_.empty()) {
            geometry_msgs::msg::Pose2D first = goal_queue_.front();
            goal_queue_.erase(goal_queue_.begin());
            goal_x_ = first.x;
            goal_y_ = first.y;
            goal_theta_ = first.theta;
            goal_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting goal: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_theta_);
        }
    }
        
    void timer_callback() {
        if (!goal_received_) return;

        // Get robot pose from TF (map -> base_footprint)
        // Localization is REQUIRED for map-based navigation
        try {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0));

            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;

            // Extract yaw from quaternion
            double siny_cosp = 2 * (transform.transform.rotation.w * transform.transform.rotation.z +
                                    transform.transform.rotation.x * transform.transform.rotation.y);
            double cosy_cosp = 1 - 2 * (transform.transform.rotation.y * transform.transform.rotation.y +
                                        transform.transform.rotation.z * transform.transform.rotation.z);
            current_theta_ = std::atan2(siny_cosp, cosy_cosp);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Localization required! Cannot get map->base_footprint transform: %s", ex.what());
            return;
        }

        static rclcpp::Time t0;
        if (t0.nanoseconds() == 0) t0 = this->now();

        // Calculate errors
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_error = normalize_angle(target_angle - current_theta_);

        // Signed errors for logging (relative to heading)
        double heading_dx = std::cos(current_theta_);
        double heading_dy = std::sin(current_theta_);
        double signed_distance_error = dx * heading_dx + dy * heading_dy;
        double signed_angle_error = angle_error;

        // PID gains
        // TODO #4: Tune these gains for better performance
        double Kp_lin = 1.0, Ki_lin = 0.0, Kd_lin = 0.0;
        double Kp_ang = 1.0, Ki_ang = 0.0, Kd_ang = 0.0;
        double dt = 0.1;
        double dist_thresh = 0.05;
        double angle_thresh = 0.1;

        geometry_msgs::msg::Twist cmd;

        // STATE 1: Rotate towards goal
        if (distance > dist_thresh && std::fabs(angle_error) > angle_thresh) {
            // TODO #1: Implement the rotation state
        }
        // STATE 2: Translate to goal
        else if (distance > dist_thresh) {
            // TODO #2: Implement the translation state
        }
        // STATE 3: Rotate to final orientation
        else {
            angle_error = normalize_angle(goal_theta_ - current_theta_);
            signed_angle_error = angle_error;

            if (std::fabs(angle_error) > angle_thresh) {
                // TODO #3: Implement the final orientation state

            } else {
                // Goal complete
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;

                RCLCPP_INFO(this->get_logger(), "Goal reached.");

                // Load next goal
                if (!goal_queue_.empty()) {
                    geometry_msgs::msg::Pose2D next = goal_queue_.front();
                    goal_queue_.erase(goal_queue_.begin());
                    goal_x_ = next.x;
                    goal_y_ = next.y;
                    goal_theta_ = next.theta;
                    RCLCPP_INFO(this->get_logger(), "Next goal: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_theta_);
                } else {
                    goal_received_ = false;
                }
            }
        }

        // Safety limits
        cmd.linear.x = std::clamp(cmd.linear.x, -0.3, 0.3);
        cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);
        cmd_vel_publisher_->publish(cmd);

        // Log
        rclcpp::Time now = this->now();
        log_file_ << (now - t0).seconds() << ","
                  << signed_distance_error << ","
                  << signed_angle_error << ","
                  << cmd.linear.x << ","
                  << cmd.angular.z << "\n";

        static int log_count = 0;
        if (++log_count % 10 == 0) log_file_.flush();
    }
        
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffMotionController>());
    rclcpp::shutdown();
    return 0;
}
