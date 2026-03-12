#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DiffMotionController : public rclcpp::Node {
public:
    DiffMotionController() : Node("diff_motion_controller") {
        // Declare parameter for localization mode
        this->declare_parameter<bool>("use_localization", false);
        use_localization_ = this->get_parameter("use_localization").as_bool();

        if (use_localization_) {
            // use TF for localization
            RCLCPP_INFO(this->get_logger(), "Using LOCALIZATION mode (TF map->base_footprint)");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        } else {
            // use odometry
            RCLCPP_INFO(this->get_logger(), "Using ODOMETRY mode (/odom topic)");
            rclcpp::QoS odom_qos = rclcpp::SensorDataQoS();
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", odom_qos, std::bind(&DiffMotionController::odom_callback, this, _1));
        }

        // Subscribe to /waypoints topic
        goal_subscriber_ = this->create_subscription<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", 10, std::bind(&DiffMotionController::goal_callback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&DiffMotionController::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "DiffMotionController initialized");
    }

private:
    bool use_localization_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<mbot_interfaces::msg::Pose2DArray>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

    // Shared thresholds for goal reaching
    const double dist_thresh_ = 0.05;
    const double angle_thresh_ = 0.075; // 0.1; // changed to be a liitle more accurate

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void goal_callback(const mbot_interfaces::msg::Pose2DArray::SharedPtr msg) {
        if (msg->poses.empty()) return;

        goal_queue_.clear();
        for (const auto& pose : msg->poses) {
            goal_queue_.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu goals", msg->poses.size());

        // Skip waypoints we're already at
        while (!goal_queue_.empty()) {
            const auto& first = goal_queue_.front();
            double dx = first.x - current_x_;
            double dy = first.y - current_y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < dist_thresh_) {
                goal_queue_.erase(goal_queue_.begin());
            } else {
                break;
            }
        }

        // Load the first unreached goal
        if (!goal_queue_.empty()) {
            geometry_msgs::msg::Pose2D first = goal_queue_.front();
            goal_queue_.erase(goal_queue_.begin());
            goal_x_ = first.x;
            goal_y_ = first.y;
            goal_theta_ = first.theta;
            goal_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting goal: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_theta_);
        } else {
            goal_received_ = false;
        }
    }

    void timer_callback() {
        if (!goal_received_) return;

        // Update pose based on mode
        if (use_localization_) {
            // Get robot pose from TF (map -> base_footprint)
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
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "Localization required! Cannot get map->base_footprint transform: %s", ex.what());
                return;
            }
        }
        // else: pose is already updated by odom_callback

        // Calculate errors
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_error = normalize_angle(target_angle - current_theta_);

        // PID gains
        // TODO #4: Tune these gains for better performance
        double Kp_lin = 1.5, Ki_lin = 0.0, Kd_lin = 0.0;
        double Kp_ang = 3.5, Ki_ang = 0.0, Kd_ang = 0.1;
        double dt = 0.05;             // Match 50ms timer period
        double integral_limit = 1.0;  // Windup protection

        geometry_msgs::msg::Twist cmd;

        // STATE 1: Turn toward target
        if (distance > dist_thresh_ && std::fabs(angle_error) > angle_thresh_) {
            // TODO #1: Implement the rotation state
            // Angular PID
            ang_error_sum_ += angle_error * dt;
            ang_error_sum_ = std::clamp(ang_error_sum_, -integral_limit, integral_limit);  // anti-windup
            double ang_deriv = (angle_error - ang_error_last_) / dt;

            cmd.linear.x = 0.0;  // Stay in place
            cmd.angular.z = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * ang_deriv;

            ang_error_last_ = angle_error;
        }
        // STATE 2: Drive forward with heading correction
        else if (distance > dist_thresh_) {
            // TODO #2: Implement the translation state
            // Linear PID
            lin_error_sum_ += distance * dt;
            lin_error_sum_ = std::clamp(lin_error_sum_, -integral_limit, integral_limit);
            double lin_deriv = (distance - lin_error_last_) / dt;

            // Angular PID (to keep pointing at the target while moving)
            ang_error_sum_ += angle_error * dt;
            double ang_deriv = (angle_error - ang_error_last_) / dt;

            cmd.linear.x = Kp_lin * distance + Ki_lin * lin_error_sum_ + Kd_lin * lin_deriv;
            cmd.angular.z = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * ang_deriv;

            lin_error_last_ = distance;
            ang_error_last_ = angle_error;
        }
        // STATE 3: Rotate to final orientation
        else {
            angle_error = normalize_angle(goal_theta_ - current_theta_);

            if (std::fabs(angle_error) > angle_thresh_) {
                // TODO #3: Implement the final orientation state
                // Angular PID
                ang_error_sum_ += angle_error * dt;
                ang_error_sum_ = std::clamp(ang_error_sum_, -integral_limit, integral_limit);
                double ang_deriv = (angle_error - ang_error_last_) / dt;

                cmd.linear.x = 0.0;
                cmd.angular.z = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * ang_deriv;

                ang_error_last_ = angle_error;
            } else {
                // Goal complete
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                lin_error_sum_ = 0.0;
                lin_error_last_ = 0.0;
                ang_error_sum_ = 0.0;
                ang_error_last_ = 0.0;

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
        cmd.linear.x = std::clamp(cmd.linear.x, -0.2, 0.2); // changed
        cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0); // changed
        cmd_vel_publisher_->publish(cmd);
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffMotionController>());
    rclcpp::shutdown();
    return 0;
}