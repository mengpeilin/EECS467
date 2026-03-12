#include "exploration_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

ExplorationNode::ExplorationNode()
: Node("exploration_node")
{
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
        std::bind(&ExplorationNode::mapCallback, this, std::placeholders::_1));

    // Timer to run frontier exploration
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0),
        std::bind(&ExplorationNode::timerCallback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Get initial origin pose from TF
    geometry_msgs::msg::Pose2D initial_pose;
    if (getRobotPose(initial_pose)) {
        origin_pose_ = initial_pose;
        RCLCPP_INFO(this->get_logger(), "ExplorationNode initialized with origin pose: (%.2f, %.2f, %.2f)",
                    origin_pose_->x, origin_pose_->y, origin_pose_->theta);
    } else {
        RCLCPP_WARN(this->get_logger(), "ExplorationNode initialized but could not get initial pose from TF. Will retry on first timer callback.");
    }
}

void ExplorationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = msg;
    dist_grid_.computeDistFromMap(*msg);
}

void ExplorationNode::timerCallback()
{
    if (!latest_map_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for map...");
        return;
    }

    geometry_msgs::msg::Pose2D robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;
    }

    // Retry getting origin pose if we don't have it yet
    if (!origin_pose_) {
        geometry_msgs::msg::Pose2D initial_pose;
        if (getRobotPose(initial_pose)) {
            origin_pose_ = initial_pose;
            RCLCPP_INFO(this->get_logger(), "Origin pose now available: (%.2f, %.2f, %.2f)",
                        origin_pose_->x, origin_pose_->y, origin_pose_->theta);
        }
        return;  // Wait for origin pose before starting exploration
    }

    // Check if current goal was reached
    if (current_goal_) {
        double dx = robot_pose.x - current_goal_->x;
        double dy = robot_pose.y - current_goal_->y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= goal_reached_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Goal reached.");
            current_goal_.reset(); // set current_goal_ empty
        } else {
            // Still navigating to current goal - replan with current robot pose
            planPath();
            publishFrontierMarkers();
            return;
        }
    }

    // If returning and reached origin, exploration is complete
    if (is_returning_) {
        double dx = robot_pose.x - origin_pose_->x;
        double dy = robot_pose.y - origin_pose_->y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= goal_reached_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Returned to origin. Exploration complete.");
        } else {
            // Still returning - replan with current robot pose
            planPath();
            publishFrontierMarkers();
        }
        return;  // Stay idle after returning to origin
    }

    //Find a valid frontier, we call selectGoal() here
    // Try to find a frontier goal
    auto goal = frontier_explorer_.selectGoal(robot_pose, dist_grid_);
    publishFrontierMarkers();
    if (!goal) {
        // No frontier found - increment counter and give it another chance
        no_frontier_count_++;
        RCLCPP_WARN(this->get_logger(), "No frontier found (%d/%d). Waiting for map update...",
                    no_frontier_count_, no_frontier_threshold_);

        // Only return to origin after multiple consecutive failures
        if (no_frontier_count_ >= no_frontier_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Exploration done. Returning to origin.");
            current_goal_ = origin_pose_.value();
            is_returning_ = true;
            no_frontier_count_ = 0;  // Reset counter
            planPath();
        }
        return;
    }

    // Found a frontier - reset the counter and proceed
    no_frontier_count_ = 0;

    // Found a frontier - plan path to it
    current_goal_ = goal.value();
    planPath();
}

void ExplorationNode::planPath()
{
    if (!latest_map_) {
        RCLCPP_WARN(this->get_logger(), "No map available yet");
        return;
    }

    geometry_msgs::msg::Pose2D robot_pose;
    if (!getRobotPose(robot_pose)) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose");
        return;
    }

    if (!current_goal_) {
        RCLCPP_WARN(this->get_logger(), "No goal set");
        return;
    }

    mbot_interfaces::msg::Pose2DArray path;
    bool success = astar_planner_.planPath(dist_grid_, robot_pose, current_goal_.value(), path);

    if (!success || path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to plan path from (%.2f, %.2f) to (%.2f, %.2f)",
                    robot_pose.x, robot_pose.y, current_goal_->x, current_goal_->y);
        return;
    }

    publishWaypoints(path);
    publishPath(path);
}

void ExplorationNode::publishWaypoints(const mbot_interfaces::msg::Pose2DArray& path)
{
    waypoints_pub_->publish(path);
}

void ExplorationNode::publishPath(const mbot_interfaces::msg::Pose2DArray& path)
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

        // Convert theta to quaternion
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = std::sin(pose2d.theta / 2.0);
        pose_stamped.pose.orientation.w = std::cos(pose2d.theta / 2.0);

        path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);
}

void ExplorationNode::publishFrontierMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Marker 0: all raw frontier cells as a POINTS cloud (cyan)
    {
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
        raw_marker.color.r = 0.0f;
        raw_marker.color.g = 1.0f;
        raw_marker.color.b = 1.0f;
        raw_marker.color.a = 0.8f;
        raw_marker.lifetime = rclcpp::Duration(2, 0);  // expire after 2s so stale frontiers clear

        for (const auto& pt : frontier_explorer_.getLastRawFrontiers()) {
            raw_marker.points.push_back(pt);
        }
        marker_array.markers.push_back(raw_marker);
    }

    // Marker 1: clustered frontier centroids as green spheres
    {
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
        cluster_marker.color.r = 0.0f;
        cluster_marker.color.g = 1.0f;
        cluster_marker.color.b = 0.0f;
        cluster_marker.color.a = 1.0f;
        cluster_marker.lifetime = rclcpp::Duration(2, 0);

        for (const auto& pt : frontier_explorer_.getLastClusteredFrontiers()) {
            cluster_marker.points.push_back(pt);
        }
        marker_array.markers.push_back(cluster_marker);
    }

    // Marker 2: current goal as a large yellow sphere
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
        goal_marker.pose.position.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.2;
        goal_marker.scale.y = 0.2;
        goal_marker.scale.z = 0.2;
        goal_marker.color.r = 1.0f;
        goal_marker.color.g = 1.0f;
        goal_marker.color.b = 0.0f;
        goal_marker.color.a = 1.0f;
        goal_marker.lifetime = rclcpp::Duration(2, 0);
        marker_array.markers.push_back(goal_marker);
    }

    frontier_pub_->publish(marker_array);
}

bool ExplorationNode::getRobotPose(geometry_msgs::msg::Pose2D &pose)
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
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
