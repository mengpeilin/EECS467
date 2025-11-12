#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "particle_filter.hpp"
#include "slam_utils.hpp"
#include "mapping.hpp"
#include "moving_laser_scan.hpp"
#include "obstacle_distance_grid.hpp"

#include <algorithm>
#include <tf2/utils.h>
#include <cmath>

class SLAMNode final : public rclcpp::Node
{
public:
    SLAMNode()
    : Node("mbot_slam_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      grid_(10.0, 10.0, 0.05, -5.0, -5.0)
    {
        // Particle filter
        particle_filter_ = std::make_unique<mbot_slam::ParticleFilter>();

        // Transform broadcaster for map -> odom
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/particle_cloud", 10);
        est_path_pub_ = create_publisher<nav_msgs::msg::Path>("/estimated_path", 10);
        odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom_path", 10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

        // Timer to periodically publish map
        map_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                       std::bind(&SLAMNode::publishMap, this));

        // Subscriptions
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&SLAMNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "SLAM node initialized. Waiting for first scan...");
    }
    
private:
    std::unique_ptr<mbot_slam::ParticleFilter> particle_filter_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>                   tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>        tf_listener_;

    mbot_slam::OccupancyGrid       grid_;
    ObstacleDistanceGrid dist_grid_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr est_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr map_timer_;

    nav_msgs::msg::Path est_path_;
    nav_msgs::msg::Path odom_path_;

    bool has_initialized_{false};
    geometry_msgs::msg::Pose est_pose_end;

    // Scan callback
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Lookup odometry transform
        nav_msgs::msg::Odometry odom_msg;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_base =
                tf_buffer_->lookupTransform("odom", "base_footprint", scan->header.stamp);

            odom_msg.header = tf_odom_base.header;
            odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
            odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
            odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
            odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        // Auto-initialize on first scan from odometry origin
        if (!has_initialized_) {
            try {
                particle_filter_->initializeAtPose(odom_msg.pose.pose);
                particle_filter_->resetOdometry(odom_msg);

                est_pose_end = odom_msg.pose.pose;

                // Initialize grid with initial scan
                MovingLaserScan initial_scan(*scan, odom_msg.pose.pose, odom_msg.pose.pose);
                updateGridWithScan(initial_scan);

                has_initialized_ = true;
                RCLCPP_INFO(get_logger(), "Particle filter initialized at odometry origin (0, 0)");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Failed to lookup odom transform at init: %s", ex.what());
                return;
            }
        } else {
            // TODO #1: Particle filter - check particle_filter.cpp TODOs
            const auto est_pose = particle_filter_->update(odom_msg, *scan, dist_grid_);

            // TODO #2: fill up the TODOs in moving_laser_scan.cpp
            MovingLaserScan interpolated_scan(*scan, est_pose, est_pose);               // works

            // TODO #6: calcualte the est_pose_end, the est_pose at the end of the lidar scan
            // Hints: use message's member time_increment to calcualte the time duration
            //          then you can get where the odometry at when the lidar's last ray fires
            //          odom_end time stamp = scan->header.stamp + lidar time duration
            //          then you will get est_pose_end = odometry_end + position corretction
            //          and the position correction is also what we broadcast map -> odom
            // MovingLaserScan interpolated_scan(*scan, est_pose, est_pose_end);        // optimal
            updateGridWithScan(interpolated_scan);

            broadcastTf(est_pose, odom_msg.pose.pose, scan->header.stamp);

            publishPose(est_pose, scan->header.stamp);
            publishCloud(scan->header.stamp);
            publishEstPath(est_pose, scan->header.stamp);
            publishOdomPath(odom_msg.pose.pose, scan->header.stamp);
        }
    }

    void updateGridWithScan(const MovingLaserScan &scan)
    {
        for (const auto& ray : scan)
        {
            // TODO #3: Fill up the TODOs in mapping.cpp
            auto ray_cells = bresenhamRayTrace(
                ray.origin.x, ray.origin.y, ray.theta, ray.range, grid_);
                
            if (!ray_cells.empty()) {
                // TODO #4: update the grid based on ray_cells here
                // We have a vector of cells along the ray, how to mark them?
                // Hints: use grid_.markCellFree(x_idx, y_idx) and grid_.markCellOccupied(x_idx, y_idx)
            }
        }

        // TODO #5: Obstacle distance grid - obstacle_distance_grid.cpp
        dist_grid_.computeDistFromMap(buildMapMessage());
    }
    
    nav_msgs::msg::OccupancyGrid buildMapMessage()
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
        return map_msg;
    }

    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid map_msg = buildMapMessage();
        map_pub_->publish(map_msg);
    }

    // Helpers: publish pose/cloud
    void publishPose(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp) const
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "map";
        msg.pose.pose       = pose;

        pose_pub_->publish(msg);
    }

    void publishCloud(const rclcpp::Time &stamp) const
    {
        const auto &particles = particle_filter_->particleCloud().particles;

        // Downsample: show only every Nth particle
        static constexpr int DOWNSAMPLE = 2;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = stamp;
        cloud.height = 1;
        cloud.width = (particles.size() + DOWNSAMPLE - 1) / DOWNSAMPLE;

        // Set fields: X, Y, Z
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(cloud.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (size_t i = 0; i < particles.size(); i += DOWNSAMPLE) {
            *iter_x = particles[i].pose.position.x;
            *iter_y = particles[i].pose.position.y;
            *iter_z = particles[i].pose.position.z + 0.02;  // slight offset for visibility
            ++iter_x; ++iter_y; ++iter_z;
        }

        cloud_pub_->publish(cloud);
    }

    void publishEstPath(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp)
    {
        est_path_.header.frame_id = "map";
        est_path_.header.stamp = stamp;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = stamp;
        pose_stamped.pose = pose;
        est_path_.poses.push_back(pose_stamped);

        est_path_pub_->publish(est_path_);
    }

    void publishOdomPath(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp)
    {
        odom_path_.header.frame_id = "odom";
        odom_path_.header.stamp = stamp;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.header.stamp = stamp;
        pose_stamped.pose = pose;
        odom_path_.poses.push_back(pose_stamped);

        odom_path_pub_->publish(odom_path_);
    }

    void broadcastTf(const geometry_msgs::msg::Pose  &map_base,
                     const geometry_msgs::msg::Pose  &odom_base,
                     const rclcpp::Time              &stamp) const
    {
        tf2::Transform tf_map_base;
        tf2::fromMsg(map_base, tf_map_base);

        tf2::Transform tf_odom_base;
        tf2::fromMsg(odom_base, tf_odom_base);

        const tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp    = stamp;
        out.header.frame_id = "map";
        out.child_frame_id  = "odom";
        out.transform       = tf2::toMsg(tf_map_odom);

        tf_broadcaster_->sendTransform(out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMNode>());
    rclcpp::shutdown();
    return 0;
}
