/**
 * Fast Exploration Node for Maze Speed Challenge
 * 
 * Optimized for:
 * - Rapid frontier selection
 * - Aggressive goal switching
 * - Greedy nearest-frontier strategy
 * - Faster replanning cycle
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "frontier_explorer.hpp"
#include "astar.hpp"
#include "obstacle_distance_grid.hpp"

#include <memory>
#include <optional>
#include <cmath>
#include <chrono>

class FastExplorationNode : public rclcpp::Node
{
public:
    FastExplorationNode() : Node("fast_exploration_node")
    {
        // Declare parameters
        this->declare_parameter<double>("goal_reached_threshold", 0.25);
        this->declare_parameter<double>("timer_period", 0.05);  // Faster than default 1.0s
        this->declare_parameter<int>("no_frontier_threshold", 15);  // Fewer retries
        this->declare_parameter<double>("min_frontier_distance", 0.25);
        this->declare_parameter<double>("goal_clearance", 0.20);

        goal_reached_threshold_ = this->get_parameter("goal_reached_threshold").as_double();
        double timer_period = this->get_parameter("timer_period").as_double();
        no_frontier_threshold_ = this->get_parameter("no_frontier_threshold").as_int();
        
        // Configure frontier explorer for speed
        frontier_explorer_.min_frontier_distance = this->get_parameter("min_frontier_distance").as_double();
        frontier_explorer_.goal_clearance = this->get_parameter("goal_clearance").as_double();
        frontier_explorer_.frontier_cluster_distance = 0.2;  // Larger clusters

        // Publishers
        waypoints_pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);

        frontier_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontier_markers", 10);

        // Subscriptions
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&FastExplorationNode::mapCallback, this, std::placeholders::_1));

        // Faster timer for rapid exploration
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&FastExplorationNode::timerCallback, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), 
            "FastExplorationNode initialized (timer: %.2fs, goal_thresh: %.2f)", 
            timer_period, goal_reached_threshold_);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
        dist_grid_.computeDistFromMap(*msg);
    }

    void timerCallback()
    {
        if (!latest_map_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for map...");
            return;
        }

        geometry_msgs::msg::Pose2D robot_pose;
        if (!getRobotPose(robot_pose)) {
            return;
        }

        // Get origin on first call
        if (!origin_pose_) {
            origin_pose_ = robot_pose;
            RCLCPP_INFO(this->get_logger(), "Origin set: (%.2f, %.2f)", 
                        origin_pose_->x, origin_pose_->y);
            return;
        }

        // Check if we've completed exploration
        if (exploration_complete_) {
            return;
        }

        // Check if returning to origin
        if (is_returning_) {
            double dist_to_origin = std::sqrt(
                std::pow(robot_pose.x - origin_pose_->x, 2) +
                std::pow(robot_pose.y - origin_pose_->y, 2));

            if (dist_to_origin <= goal_reached_threshold_) {
                RCLCPP_INFO(this->get_logger(), "=== EXPLORATION COMPLETE! Returned to origin ===");
                exploration_complete_ = true;
                return;
            }

            // Replan path to origin periodically
            planPath();
            publishFrontierMarkers();
            return;
        }

        // Check if current goal reached
        if (current_goal_) {
            double dist = std::sqrt(
                std::pow(robot_pose.x - current_goal_->x, 2) +
                std::pow(robot_pose.y - current_goal_->y, 2));

            if (dist <= goal_reached_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached, selecting next frontier");
                current_goal_.reset();
            } else {
                // Still navigating - replan with updated position
                planPath();
                publishFrontierMarkers();
                return;
            }
        }

        // Find next frontier
        auto goal = frontier_explorer_.selectGoal(robot_pose, dist_grid_);
        publishFrontierMarkers();

        if (!goal) {
            no_frontier_count_++;
            RCLCPP_WARN(this->get_logger(), "No frontier (%d/%d)", 
                        no_frontier_count_, no_frontier_threshold_);

            if (no_frontier_count_ >= no_frontier_threshold_) {
                RCLCPP_INFO(this->get_logger(), "=== EXPLORATION DONE! Returning to origin ===");
                current_goal_ = origin_pose_;
                is_returning_ = true;
                no_frontier_count_ = 0;
                planPath();
            }
            return;
        }

        no_frontier_count_ = 0;
        current_goal_ = goal;
        RCLCPP_INFO(this->get_logger(), "New frontier goal: (%.2f, %.2f)", 
                    current_goal_->x, current_goal_->y);
        planPath();
    }

    void planPath()
    {
        if (!latest_map_ || !current_goal_) return;

        geometry_msgs::msg::Pose2D robot_pose;
        if (!getRobotPose(robot_pose)) return;

        mbot_interfaces::msg::Pose2DArray path;
        bool success = astar_planner_.planPath(dist_grid_, robot_pose, current_goal_.value(), path);

        if (!success || path.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path planning failed!");
            // If planning fails, try a different frontier next time
            current_goal_.reset();
            return;
        }

        // Downsample waypoints for faster execution (every Nth point)
        mbot_interfaces::msg::Pose2DArray sparse_path;
        int step = 3;  // Keep every 3rd waypoint
        for (size_t i = 0; i < path.poses.size(); i += step) {
            sparse_path.poses.push_back(path.poses[i]);
        }
        // Always include the final goal
        if (sparse_path.poses.empty() || 
            (sparse_path.poses.back().x != path.poses.back().x ||
             sparse_path.poses.back().y != path.poses.back().y)) {
            sparse_path.poses.push_back(path.poses.back());
        }

        waypoints_pub_->publish(sparse_path);
        publishPath(path);  // Publish full path for visualization
    }

    void publishPath(const mbot_interfaces::msg::Pose2DArray& path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& pose2d : path.poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = pose2d.x;
            pose_stamped.pose.position.y = pose2d.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.z = std::sin(pose2d.theta / 2.0);
            pose_stamped.pose.orientation.w = std::cos(pose2d.theta / 2.0);
            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_msg);
    }

    void publishFrontierMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Raw frontiers (cyan)
        visualization_msgs::msg::Marker raw_marker;
        raw_marker.header.frame_id = "map";
        raw_marker.header.stamp = this->now();
        raw_marker.ns = "frontier_raw";
        raw_marker.id = 0;
        raw_marker.type = visualization_msgs::msg::Marker::POINTS;
        raw_marker.action = visualization_msgs::msg::Marker::ADD;
        raw_marker.pose.orientation.w = 1.0;
        raw_marker.scale.x = 0.05;
        raw_marker.scale.y = 0.05;
        raw_marker.color.g = 1.0f;
        raw_marker.color.b = 1.0f;
        raw_marker.color.a = 0.8f;
        raw_marker.lifetime = rclcpp::Duration(1, 0);
        for (const auto& pt : frontier_explorer_.getLastRawFrontiers()) {
            raw_marker.points.push_back(pt);
        }
        marker_array.markers.push_back(raw_marker);

        // Clustered centroids (green)
        visualization_msgs::msg::Marker cluster_marker;
        cluster_marker.header.frame_id = "map";
        cluster_marker.header.stamp = this->now();
        cluster_marker.ns = "frontier_clusters";
        cluster_marker.id = 1;
        cluster_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        cluster_marker.action = visualization_msgs::msg::Marker::ADD;
        cluster_marker.pose.orientation.w = 1.0;
        cluster_marker.scale.x = 0.12;
        cluster_marker.scale.y = 0.12;
        cluster_marker.scale.z = 0.12;
        cluster_marker.color.g = 1.0f;
        cluster_marker.color.a = 1.0f;
        cluster_marker.lifetime = rclcpp::Duration(1, 0);
        for (const auto& pt : frontier_explorer_.getLastClusteredFrontiers()) {
            cluster_marker.points.push_back(pt);
        }
        marker_array.markers.push_back(cluster_marker);

        // Current goal (yellow)
        if (current_goal_) {
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = "map";
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "frontier_goal";
            goal_marker.id = 2;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            goal_marker.pose.position.x = current_goal_->x;
            goal_marker.pose.position.y = current_goal_->y;
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = 0.2;
            goal_marker.scale.y = 0.2;
            goal_marker.scale.z = 0.2;
            goal_marker.color.r = 1.0f;
            goal_marker.color.g = 1.0f;
            goal_marker.color.a = 1.0f;
            goal_marker.lifetime = rclcpp::Duration(1, 0);
            marker_array.markers.push_back(goal_marker);
        }

        frontier_pub_->publish(marker_array);
    }

    bool getRobotPose(geometry_msgs::msg::Pose2D &pose)
    {
        try {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);

            pose.x = tf.transform.translation.x;
            pose.y = tf.transform.translation.y;

            tf2::Quaternion q(
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            pose.theta = yaw;

            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::optional<geometry_msgs::msg::Pose2D> current_goal_;
    double goal_reached_threshold_ = 0.25;

    // Planners
    mbot_nav::AStarPlanner astar_planner_;
    mbot_nav::FrontierExplorer frontier_explorer_;
    ObstacleDistanceGrid dist_grid_;

    // State
    std::optional<geometry_msgs::msg::Pose2D> origin_pose_;
    bool is_returning_ = false;
    bool exploration_complete_ = false;
    int no_frontier_count_ = 0;
    int no_frontier_threshold_ = 10;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastExplorationNode>());
    rclcpp::shutdown();
    return 0;
}
