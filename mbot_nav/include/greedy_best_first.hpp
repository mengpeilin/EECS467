#pragma once

#include "obstacle_distance_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include <vector>
#include <unordered_map>
#include <queue>

namespace mbot_nav {

struct GreedyNode {
    int x, y;
    float priority;  // g + h weighted priority for better pathfinding
    int depth;       // distance from start for cost calculation
    bool operator>(const GreedyNode& other) const {
        return priority > other.priority;
    }
};

class GreedyBestFirstPlanner{
public:
    GreedyBestFirstPlanner();

    bool planPath(const ObstacleDistanceGrid& dist_grid,
                  const geometry_msgs::msg::Pose2D& start,
                  const geometry_msgs::msg::Pose2D& goal,
                  mbot_interfaces::msg::Pose2DArray& path);

private:
    float heuristic(int x1, int y1, int x2, int y2);

    std::vector<std::pair<int, int>> reconstructPath(
        const std::unordered_map<int, int>& came_from, int goal_idx, int width);

    mbot_interfaces::msg::Pose2DArray smoothPath(
        const ObstacleDistanceGrid& dist_grid,
        const mbot_interfaces::msg::Pose2DArray& raw_path);

    bool isLineFree(const ObstacleDistanceGrid& dist_grid,
                    float x0, float y0, float x1, float y1);

    int toIndex(int x, int y, int width) const { return y * width + x; }
};

}  // namespace mbot_nav
