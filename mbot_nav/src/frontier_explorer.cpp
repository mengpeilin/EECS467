#include "frontier_explorer.hpp"
#include <queue>
#include <limits>
#include <cmath>
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
    last_raw_frontiers_ = raw_frontiers;

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
    last_clustered_frontiers_ = clustered;

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
    // robot_gx/robot_gy currently unused (you can use them later for BFS-limited search)
    (void)robot_gx;
    (void)robot_gy;

    const int width = dist_grid.getWidth();
    const int height = dist_grid.getHeight();

    std::vector<geometry_msgs::msg::Point> frontier_points;
    frontier_points.reserve(1024);

    // TODO #1: find all the frontier_points
    //  utilize the function isFrontierCell to check if cell is a frontier
    //  utilize the function gridToWorld to convert grid coordinates to world
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (isFrontierCell(x, y, dist_grid)) {
                frontier_points.push_back(gridToWorld(x, y, dist_grid));
            }
        }
    }

    return frontier_points;
}

std::vector<geometry_msgs::msg::Point> FrontierExplorer::clusterFrontiers(
    const std::vector<geometry_msgs::msg::Point> &frontiers)
{
    std::vector<geometry_msgs::msg::Point> clustered;
    if (frontiers.empty()) return clustered;

    const double thr2 = frontier_cluster_distance * frontier_cluster_distance;
    std::vector<bool> visited(frontiers.size(), false);

    // TODO #3: cluster the frontiers nearby
    //  Hint: when frontiers are all clustered together,
    //  we can calculate a centroid within a distance
    //  utilize the parameters in the header file => frontier_cluster_distance
    for (size_t i = 0; i < frontiers.size(); ++i) {
        if (visited[i]) continue;

        // BFS/queue over points within threshold
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        double sum_x = 0.0, sum_y = 0.0;
        int count = 0;

        while (!q.empty()) {
            size_t idx = q.front();
            q.pop();

            sum_x += frontiers[idx].x;
            sum_y += frontiers[idx].y;
            count++;

            for (size_t j = 0; j < frontiers.size(); ++j) {
                if (visited[j]) continue;
                double dx = frontiers[j].x - frontiers[idx].x;
                double dy = frontiers[j].y - frontiers[idx].y;
                if (dx * dx + dy * dy <= thr2) {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }

        geometry_msgs::msg::Point c;
        c.x = sum_x / std::max(1, count);
        c.y = sum_y / std::max(1, count);
        c.z = 0.0;
        clustered.push_back(c);
    }

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

    // TODO #2: frontier must be adjacent to unknown cell (8-connectivity)
    const int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1};
    const int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1};

    for (int k = 0; k < 8; ++k) {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (!dist_grid.isCellInGrid(nx, ny)) continue;

        // Unknown in OccupancyGrid is typically -1
        if (dist_grid.getOccupancy(nx, ny) == -1) {
            return true;
        }
    }

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