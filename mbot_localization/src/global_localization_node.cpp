/**
 * Global Localization Node
 *
 * Localization with unknown starting position on a known map.
 * - Initializes particles uniformly across all free space
 * - Runs particle filter with LiDAR sensor updates
 * - Uses simple obstacle avoidance to explore safely
 * - Detects convergence via particle variance
 * - Once localized, publishes TF and stops exploration motion
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "particle_filter.hpp"
#include "obstacle_distance_grid.hpp"
#include "localization_utils.hpp"

#include <algorithm>
#include <cmath>

class GlobalLocalizationNode final : public rclcpp::Node
{
public:
    GlobalLocalizationNode()
    : Node("global_localization_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Parameters
        this->declare_parameter("variance_threshold", 0.05);
        this->declare_parameter("forward_speed", 0.1);
        this->declare_parameter("rotation_speed", 0.5);
        this->declare_parameter("obstacle_distance", 0.35);

        variance_threshold_ = this->get_parameter("variance_threshold").as_double();
        forward_speed_      = this->get_parameter("forward_speed").as_double();
        rotation_speed_     = this->get_parameter("rotation_speed").as_double();
        obstacle_distance_  = this->get_parameter("obstacle_distance").as_double();

        // Particle filter
        particle_filter_ = std::make_unique<mbot_localization::ParticleFilter>();

        // Transform broadcaster for map -> odom
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/particle_cloud", 10);
        est_path_pub_ = create_publisher<nav_msgs::msg::Path>("/estimated_path", 10);
        cmd_vel_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriptions
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&GlobalLocalizationNode::mapCallback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&GlobalLocalizationNode::scanCallback, this, std::placeholders::_1));

        // Timer for obstacle avoidance motion (10 Hz)
        motion_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GlobalLocalizationNode::motionTimerCallback, this));

        RCLCPP_INFO(get_logger(),
            "Global localization node started. Waiting for map...");
        RCLCPP_INFO(get_logger(),
            "  variance_threshold=%.4f, forward_speed=%.2f, rotation_speed=%.2f, obstacle_dist=%.2f",
            variance_threshold_, forward_speed_, rotation_speed_, obstacle_distance_);
    }

private:
    std::unique_ptr<mbot_localization::ParticleFilter> particle_filter_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>                   tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>        tf_listener_;

    ObstacleDistanceGrid           dist_grid_;
    nav_msgs::msg::OccupancyGrid   map_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr est_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr motion_timer_;

    nav_msgs::msg::Path est_path_;

    // State flags
    bool have_map_{false};
    bool initialized_{false};
    bool localized_{false};

    // Latest scan for obstacle avoidance
    sensor_msgs::msg::LaserScan latest_scan_;
    bool have_scan_{false};

    // Parameters
    double variance_threshold_;
    double forward_speed_;
    double rotation_speed_;
    double obstacle_distance_;

    // ────────────── Callbacks ──────────────

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        dist_grid_.computeDistFromMap(map_);
        have_map_ = true;

        if (!initialized_) {
            // Global initialization: scatter particles across free space
            particle_filter_->initializeGlobal(map_);
            initialized_ = true;

            // Set odom reference from current TF
            try {
                geometry_msgs::msg::TransformStamped tf_odom_base =
                    tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0));
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header = tf_odom_base.header;
                odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
                odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
                odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
                odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
                particle_filter_->resetOdometry(odom_msg);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Could not get initial odom transform: %s", ex.what());
            }

            RCLCPP_INFO(get_logger(),
                "Map received (%ux%u). Particles initialized globally. Starting exploration.",
                map_.info.width, map_.info.height);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Map updated (%ux%u)", map_.info.width, map_.info.height);
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Store latest scan for obstacle avoidance
        latest_scan_ = *scan;
        have_scan_ = true;

        if (!have_map_ || !initialized_) return;

        // Lookup odometry
        nav_msgs::msg::Odometry odom_msg;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_base =
                tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0));
            odom_msg.header = tf_odom_base.header;
            odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
            odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
            odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
            odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        // Particle filter update: motion + sensor + resample
        const auto est_pose = particle_filter_->update(odom_msg, *scan, dist_grid_);

        // Compute particle variance for localization detection
        const double variance = particle_filter_->computeVariance();

        // Publish visualization
        publishPose(est_pose, scan->header.stamp);
        publishCloud(scan->header.stamp);
        publishEstPath(est_pose, scan->header.stamp);

        // Always broadcast TF (needed for navigation after localization)
        broadcastTf(est_pose, odom_msg.pose.pose, scan->header.stamp);

        // Check localization convergence
        if (!localized_) {
            if (variance < variance_threshold_) {
                localized_ = true;
                RCLCPP_INFO(get_logger(),
                    "=== LOCALIZATION SUCCESSFUL === variance=%.6f < threshold=%.6f",
                    variance, variance_threshold_);
                RCLCPP_INFO(get_logger(),
                    "Estimated pose: (%.2f, %.2f). Stopping exploration motion.",
                    est_pose.position.x, est_pose.position.y);
                RCLCPP_INFO(get_logger(),
                    "TF map->odom is now being published. Normal navigation can begin.");

                // Stop the robot
                geometry_msgs::msg::Twist stop;
                cmd_vel_pub_->publish(stop);
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Localizing... variance=%.6f (threshold=%.6f)",
                    variance, variance_threshold_);
            }
        }
    }

    // ────────────── Obstacle Avoidance Motion ──────────────

    void motionTimerCallback()
    {
        // Only move when not yet localized and we have scan data
        if (localized_ || !have_scan_ || !initialized_) return;

        geometry_msgs::msg::Twist cmd;

        // Check front region of LiDAR for obstacles
        const double front_min_dist = getMinFrontDistance(latest_scan_);

        if (front_min_dist > obstacle_distance_) {
            // Path ahead is clear: move forward slowly
            cmd.linear.x = forward_speed_;
            cmd.angular.z = 0.0;
        } else {
            // Obstacle detected: rotate in place to find a clear direction
            cmd.linear.x = 0.0;
            cmd.angular.z = rotation_speed_;
        }

        cmd_vel_pub_->publish(cmd);
    }

    double getMinFrontDistance(const sensor_msgs::msg::LaserScan& scan) const
    {
        // Check a +/- 30 degree cone in front of the robot
        // Note: LiDAR is mounted backward (180° offset), so "front" of robot
        // is at angle_min + pi region. We check the ranges near the middle
        // of the scan that correspond to the forward direction.
        const double front_half_angle = 0.52;  // ~30 degrees

        double min_dist = scan.range_max;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            // Adjust for backward-mounted LiDAR: robot forward = laser backward (angle ~= ±π)
            // Front of robot corresponds to angles near ±π in laser frame
            double abs_angle = std::abs(angle);
            bool is_front = (abs_angle > (M_PI - front_half_angle));

            if (is_front) {
                double r = scan.ranges[i];
                if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
                    min_dist = std::min(min_dist, static_cast<double>(r));
                }
            }
        }

        return min_dist;
    }

    // ────────────── Publishing Helpers ──────────────

    void publishPose(const geometry_msgs::msg::Pose& pose, const rclcpp::Time& stamp) const
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = map_.header.frame_id;
        msg.pose.pose       = pose;
        pose_pub_->publish(msg);
    }

    void publishCloud(const rclcpp::Time& stamp) const
    {
        const auto& particles = particle_filter_->particleCloud().particles;
        static constexpr int DOWNSAMPLE = 2;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = map_.header.frame_id;
        cloud.header.stamp = stamp;
        cloud.height = 1;
        cloud.width = (particles.size() + DOWNSAMPLE - 1) / DOWNSAMPLE;

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
            *iter_z = particles[i].pose.position.z + 0.02;
            ++iter_x; ++iter_y; ++iter_z;
        }

        cloud_pub_->publish(cloud);
    }

    void publishEstPath(const geometry_msgs::msg::Pose& pose, const rclcpp::Time& stamp)
    {
        est_path_.header.frame_id = map_.header.frame_id;
        est_path_.header.stamp = stamp;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_.header.frame_id;
        pose_stamped.header.stamp = stamp;
        pose_stamped.pose = pose;
        est_path_.poses.push_back(pose_stamped);

        est_path_pub_->publish(est_path_);
    }

    void broadcastTf(const geometry_msgs::msg::Pose& map_base,
                     const geometry_msgs::msg::Pose& odom_base,
                     const rclcpp::Time& stamp) const
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
