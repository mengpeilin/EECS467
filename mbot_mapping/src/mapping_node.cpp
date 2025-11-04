#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "mapping.hpp"
#include "moving_laser_scan.hpp"
#include <geometry_msgs/msg/pose.hpp>


class MappingNode : public rclcpp::Node
{
public:
    MappingNode()
    : Node("mbot_mapping_node"),
      grid_(10.0, 10.0, 0.05, -5.0, -5.0),  // 10x10m map, 5cm resolution, origin at center (-5,-5) to (5,5)
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        RCLCPP_INFO(this->get_logger(), "MappingNode will start in 1 second...");

        // Delay initialization by 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

        double publish_rate = 1.0; // Hz
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate), std::bind(&MappingNode::publishMap, this));
    }

private:
    OccupancyGrid grid_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    geometry_msgs::msg::Pose previous_pose_;
    bool initialized_ = false;

    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;

        // Lookup odometry transform at scan timestamp
        geometry_msgs::msg::Pose current_pose;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_lidar =
                tf_buffer_->lookupTransform("odom", "lidar_link", msg->header.stamp);
            current_pose.position.x = tf_odom_lidar.transform.translation.x;
            current_pose.position.y = tf_odom_lidar.transform.translation.y;
            current_pose.position.z = tf_odom_lidar.transform.translation.z;
            current_pose.orientation = tf_odom_lidar.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        if (!initialized_) {
            previous_pose_ = current_pose;
            initialized_ = true;
            return;
        }

        float dx = current_pose.position.x - previous_pose_.position.x;
        float dy = current_pose.position.y - previous_pose_.position.y;
        float dist_moved = std::sqrt(dx * dx + dy * dy);

        float yaw_prev = yawFromQuat(previous_pose_.orientation);
        float yaw_curr = yawFromQuat(current_pose.orientation);
        float dtheta = std::fabs(yaw_curr - yaw_prev);
        if (dtheta > M_PI) dtheta = 2 * M_PI - dtheta;

        if (dist_moved < 0.01 && dtheta < 0.05f) {
            // TODO #1: fill up the TODOs in moving_laser_scan.cpp
            MovingLaserScan scan(*msg, current_pose, current_pose);
            updateGrid(scan);
        } else {
            MovingLaserScan scan(*msg, previous_pose_, current_pose);
            updateGrid(scan);
        }

        previous_pose_ = current_pose;
    }

    void updateGrid(const MovingLaserScan& scan)
    {
        for (const auto& ray : scan)
        {
            // TODO #2: Fill up the TODOs in mapping.cpp
            auto ray_cells = bresenhamRayTrace(
                ray.origin.x, ray.origin.y, ray.theta, ray.range, grid_);
                
            if (!ray_cells.empty()) {
                // TODO #3: update the grid based on ray_cells here
                // We have a vector of cells along the ray, how to mark them?
                // Hints: use grid_.markCellFree(x_idx, y_idx) and grid_.markCellOccupied(x_idx, y_idx)
            }
        }
    }
    
    float yawFromQuat(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return static_cast<float>(yaw);
    }

    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();

        map_msg.info.resolution = grid_.getResolution();
        map_msg.info.width = grid_.getWidth();
        map_msg.info.height = grid_.getHeight();
        map_msg.info.origin.position.x = grid_.getOriginX();
        map_msg.info.origin.position.y = grid_.getOriginY();
        map_msg.info.origin.position.z = 0;
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data = grid_.getOccupancyGrid();
        map_pub_->publish(map_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}