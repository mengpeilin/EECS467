/**
 * High-Speed Motion Controller for Maze Challenge
 * 
 * Key features:
 * - Pure pursuit / carrot-following for smooth trajectory
 * - Higher velocity limits
 * - Lookahead to anticipate turns
 * - Relaxed waypoint thresholds for speed
 * - Continuous motion between waypoints (no stop-rotate-go)
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <deque>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class FastMotionController : public rclcpp::Node {
public:
    FastMotionController() : Node("fast_motion_controller") {
        // Declare parameters
        this->declare_parameter<bool>("use_localization", true);
        this->declare_parameter<double>("max_linear_vel", 0.5);
        this->declare_parameter<double>("max_angular_vel", 2.5);
        this->declare_parameter<double>("lookahead_distance", 0.25);
        this->declare_parameter<double>("goal_tolerance", 0.15);
        
        use_localization_ = this->get_parameter("use_localization").as_bool();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        RCLCPP_INFO(this->get_logger(), "Fast Motion Controller - Max Lin: %.2f, Max Ang: %.2f", 
                    max_linear_vel_, max_angular_vel_);

        if (use_localization_) {
            RCLCPP_INFO(this->get_logger(), "Using LOCALIZATION mode (TF map->base_footprint)");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Using ODOMETRY mode (/odom topic)");
            rclcpp::QoS odom_qos = rclcpp::SensorDataQoS();
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", odom_qos, std::bind(&FastMotionController::odom_callback, this, _1));
        }

        // Subscribe to waypoints
        goal_subscriber_ = this->create_subscription<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", 10, std::bind(&FastMotionController::goal_callback, this, _1));

        // Subscribe to laser scan for emergency stop
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&FastMotionController::scan_callback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Faster control loop for high-speed operation (25ms = 40Hz)
        timer_ = this->create_wall_timer(25ms, std::bind(&FastMotionController::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "FastMotionController initialized");
    }

private:
    bool use_localization_;
    double max_linear_vel_;
    double max_angular_vel_;
    double lookahead_distance_;
    double goal_tolerance_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<mbot_interfaces::msg::Pose2DArray>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    float min_forward_dist_ = 1.0f;  // minimum obstacle distance in forward arc (from laser)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;

    std::deque<geometry_msgs::msg::Pose2D> waypoint_queue_;
    bool has_waypoints_ = false;
    bool final_orientation_ = false;
    double final_theta_ = 0.0;

    // For derivative calculation
    double prev_angle_error_ = 0.0;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check forward arc ±45 degrees for obstacles
        const float half_arc = M_PI / 4.0f;  // 45 degrees
        float min_dist = std::numeric_limits<float>::max();
        const int num_ranges = static_cast<int>(msg->ranges.size());
        for (int i = 0; i < num_ranges; ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            if (std::fabs(angle) > half_arc) continue;
            float r = msg->ranges[i];
            if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
                min_dist = std::min(min_dist, r);
            }
        }
        min_forward_dist_ = (min_dist == std::numeric_limits<float>::max()) ? 1.0f : min_dist;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = quaternionToYaw(msg->pose.pose.orientation);
    }

    void goal_callback(const mbot_interfaces::msg::Pose2DArray::SharedPtr msg) {
        if (msg->poses.empty()) return;

        waypoint_queue_.clear();
        for (const auto& pose : msg->poses) {
            waypoint_queue_.push_back(pose);
        }

        // Save final orientation
        final_theta_ = msg->poses.back().theta;
        final_orientation_ = false;
        has_waypoints_ = true;

        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints for high-speed navigation", 
                    msg->poses.size());

        // Skip waypoints we're already past
        prunePassedWaypoints();
    }

    void prunePassedWaypoints() {
        while (waypoint_queue_.size() > 1) {
            double dx = waypoint_queue_.front().x - current_x_;
            double dy = waypoint_queue_.front().y - current_y_;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < goal_tolerance_) {
                waypoint_queue_.pop_front();
            } else {
                break;
            }
        }
    }

    void timer_callback() {
        // Update pose
        if (use_localization_) {
            if (!updatePoseFromTF()) {
                stopRobot();
                return;
            }
        }

        if (!has_waypoints_ || waypoint_queue_.empty()) {
            if (final_orientation_) {
                // Rotate to final orientation
                double angle_error = normalize_angle(final_theta_ - current_theta_);
                if (std::fabs(angle_error) > 0.1) {
                    geometry_msgs::msg::Twist cmd;
                    cmd.linear.x = 0.0;
                    cmd.angular.z = std::clamp(2.5 * angle_error, -max_angular_vel_, max_angular_vel_);
                    cmd_vel_publisher_->publish(cmd);
                } else {
                    stopRobot();
                    final_orientation_ = false;
                    RCLCPP_INFO(this->get_logger(), "Navigation complete!");
                }
            } else {
                stopRobot();
            }
            return;
        }

        // Prune waypoints we've passed
        prunePassedWaypoints();

        if (waypoint_queue_.empty()) {
            final_orientation_ = true;
            return;
        }

        // Pure Pursuit: Find lookahead point on path
        auto [lookahead_x, lookahead_y] = findLookaheadPoint();

        // Calculate steering
        double dx = lookahead_x - current_x_;
        double dy = lookahead_y - current_y_;
        double target_angle = std::atan2(dy, dx);
        double angle_error = normalize_angle(target_angle - current_theta_);

        // Distance to current waypoint
        double dist_to_waypoint = std::sqrt(
            std::pow(waypoint_queue_.front().x - current_x_, 2) +
            std::pow(waypoint_queue_.front().y - current_y_, 2));

        // Calculate velocities
        geometry_msgs::msg::Twist cmd;

        // PID for angular velocity with derivative
        double Kp_ang = 3.0;
        double Kd_ang = 0.15;
        double dt = 0.025;  // 25ms
        double angle_deriv = (angle_error - prev_angle_error_) / dt;
        prev_angle_error_ = angle_error;

        cmd.angular.z = Kp_ang * angle_error + Kd_ang * angle_deriv;

        // Adaptive linear velocity based on:
        // 1. How well aligned we are (slow down for turns)
        // 2. Distance to goal (slow down near waypoints if it's the last one)
        double alignment_factor = std::cos(angle_error);  // 1.0 when aligned, 0 at 90 deg
        alignment_factor = std::max(0.2, alignment_factor);  // Keep moving even during turns

        // Speed reduction near final waypoint
        double slowdown_factor = 1.0;
        if (waypoint_queue_.size() == 1 && dist_to_waypoint < 0.3) {
            slowdown_factor = std::max(0.3, dist_to_waypoint / 0.3);
        }

        cmd.linear.x = max_linear_vel_ * alignment_factor * slowdown_factor;

        // If angle error is large, reduce forward speed more aggressively
        if (std::fabs(angle_error) > 0.5) {  // ~30 degrees
            cmd.linear.x *= 0.4;
        }

        // Emergency brake: limit speed based on nearest forward obstacle (from laser)
        // Stop zone: < 0.18m (robot radius), slow zone: < 0.35m
        constexpr double stop_dist  = 0.18;
        constexpr double slow_dist  = 0.35;
        if (min_forward_dist_ < stop_dist) {
            cmd.linear.x = 0.0;
        } else if (min_forward_dist_ < slow_dist) {
            double brake = (min_forward_dist_ - stop_dist) / (slow_dist - stop_dist);
            cmd.linear.x *= brake;
        }

        // Apply limits
        cmd.linear.x = std::clamp(cmd.linear.x, 0.0, max_linear_vel_);
        cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_vel_, max_angular_vel_);

        cmd_vel_publisher_->publish(cmd);

        // Check if we reached current waypoint
        if (dist_to_waypoint < goal_tolerance_) {
            waypoint_queue_.pop_front();
            if (waypoint_queue_.empty()) {
                RCLCPP_INFO(this->get_logger(), "All waypoints reached, adjusting final heading");
                final_orientation_ = true;
            }
        }
    }

    std::pair<double, double> findLookaheadPoint() {
        // Simple lookahead: project forward on path
        double accumulated_dist = 0.0;
        double prev_x = current_x_;
        double prev_y = current_y_;

        for (const auto& wp : waypoint_queue_) {
            double seg_dx = wp.x - prev_x;
            double seg_dy = wp.y - prev_y;
            double seg_dist = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

            if (accumulated_dist + seg_dist >= lookahead_distance_) {
                // Interpolate along this segment
                double remaining = lookahead_distance_ - accumulated_dist;
                double ratio = remaining / seg_dist;
                double lx = prev_x + ratio * seg_dx;
                double ly = prev_y + ratio * seg_dy;
                return {lx, ly};
            }

            accumulated_dist += seg_dist;
            prev_x = wp.x;
            prev_y = wp.y;
        }

        // If path is shorter than lookahead, use last waypoint
        return {waypoint_queue_.back().x, waypoint_queue_.back().y};
    }

    bool updatePoseFromTF() {
        try {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0));

            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            current_theta_ = quaternionToYaw(transform.transform.rotation);
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd);
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastMotionController>());
    rclcpp::shutdown();
    return 0;
}
