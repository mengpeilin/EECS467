#include "frontier_explorer.hpp"
#include <queue>
#include <limits>
#include "rclcpp/rclcpp.hpp"

namespace mbot_nav
{

FrontierExplorer::FrontierExplorer()
{
}

std::optional<geometry_msgs::msg::Pose2D> FrontierExplorer::selectGoal(
    const geometry_msgs::msg::Pose2D &robot_pose,
    const ObstacleDistanceGrid &dist_grid)
{
    // Convert robot pose to grid coordinates
    int robot_gx = static_cast<int>((robot_pose.x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
    int robot_gy = static_cast<int>((robot_pose.y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());

    // Detect frontier cells
    auto raw_frontiers = detectFrontiers(robot_gx, robot_gy, dist_grid);

    if (raw_frontiers.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "No frontiers found");
        return std::nullopt;
    }

    RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "Found %zu raw frontier points", raw_frontiers.size());

    // Filter frontiers by clearance
    std::vector<geometry_msgs::msg::Point> clearance_filtered;
    // TODO #4: tune the parameter "goal_clearance"
    for (const auto& point : raw_frontiers) {
        int gx = static_cast<int>((point.x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
        int gy = static_cast<int>((point.y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());

        if (dist_grid.isCellInGrid(gx, gy) &&
            dist_grid.getDistance(gx, gy) >= goal_clearance) {
            clearance_filtered.push_back(point);
        }
    }

    if (clearance_filtered.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("frontier_explorer"),
                    "No frontiers with sufficient clearance.");
        return std::nullopt;
    }

    // Cluster filtered frontiers by distance
    auto clustered = clusterFrontiers(clearance_filtered);

    // Pick the closest frontier cluster
    double closest_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point best_frontier;

    for (const auto& centroid : clustered) {
        double dx = centroid.x - robot_pose.x;
        double dy = centroid.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Skip if too close to robot
        // TODO #5: tune the paramter "min_frontier_distance"
        if (distance < min_frontier_distance) {
            continue;
        }

        if (distance < closest_distance) {
            closest_distance = distance;
            best_frontier = centroid;
        }
    }

    // Check if we found a valid frontier
    if (closest_distance == std::numeric_limits<double>::max()) {
        RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "No valid frontiers (all too close)");
        return std::nullopt;
    }

    // Calculate heading toward the frontier
    double dx = best_frontier.x - robot_pose.x;
    double dy = best_frontier.y - robot_pose.y;
    double approach_angle = std::atan2(dy, dx);

    geometry_msgs::msg::Pose2D goal;
    goal.x = best_frontier.x;
    goal.y = best_frontier.y;
    goal.theta = approach_angle;

    return goal;
}

std::vector<geometry_msgs::msg::Point> FrontierExplorer::detectFrontiers(
    int robot_gx, int robot_gy,
    const ObstacleDistanceGrid &dist_grid)
{
    int width = dist_grid.getWidth();
    int height = dist_grid.getHeight();
    std::vector<geometry_msgs::msg::Point> frontier_points;

    // TODO #1: find all the frontier_points
    //  utilize the function isFrontierCell to check if cell is a frontier
    //  utilize the function gridToWorld to convert grid coordinates to world







    return frontier_points;
}

std::vector<geometry_msgs::msg::Point> FrontierExplorer::clusterFrontiers(
    const std::vector<geometry_msgs::msg::Point> &frontiers)
{

    std::vector<geometry_msgs::msg::Point> clustered;

    // TODO #3: cluster the frontiers nearby
    //  Hint: when frontiers are all clustered together,
    //  we can calculate a centroid within a distance
    //  utilize the parameters in the header file => frontier_cluster_distance




    return clustered;

}

bool FrontierExplorer::isFrontierCell(int x, int y, const ObstacleDistanceGrid &dist_grid)
{
    // Must be in bounds
    if (!dist_grid.isCellInGrid(x, y)) {
        return false;
    }

    // Must be a free cell (occupancy = 0)
    if (dist_grid.getOccupancy(x, y) != 0) {
        return false;
    }


    // TODO #2: what else to define a frontier cell? (in other words, when to return true)?
    // Hint: a frontier must be adjacent to unknown cell (8-connectivity)


   

    return false;
}

geometry_msgs::msg::Point FrontierExplorer::gridToWorld(
    int x, int y,
    const ObstacleDistanceGrid &dist_grid)
{
    geometry_msgs::msg::Point pt;
    pt.x = dist_grid.getOrigin().position.x + (x + 0.5) * dist_grid.getResolution();
    pt.y = dist_grid.getOrigin().position.y + (y + 0.5) * dist_grid.getResolution();
    pt.z = 0.0;
    return pt;
}

} // namespace mbot_nav
