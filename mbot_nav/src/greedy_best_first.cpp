#include "greedy_best_first.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace mbot_nav;

GreedyBestFirstPlanner::GreedyBestFirstPlanner() {}

float GreedyBestFirstPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(static_cast<float>(x2 - x1), static_cast<float>(y2 - y1));
}

bool GreedyBestFirstPlanner::planPath(const ObstacleDistanceGrid& dist_grid,
                                      const geometry_msgs::msg::Pose2D& start,
                                      const geometry_msgs::msg::Pose2D& goal,
                                      mbot_interfaces::msg::Pose2DArray& path) {
    path.poses.clear();
    mbot_interfaces::msg::Pose2DArray raw_path;

    const int width  = dist_grid.getWidth();
    const float res  = dist_grid.getResolution();
    const auto& origin = dist_grid.getOrigin().position;

    const int start_x = static_cast<int>((start.x - origin.x) / res);
    const int start_y = static_cast<int>((start.y - origin.y) / res);
    const int goal_x  = static_cast<int>((goal.x  - origin.x) / res);
    const int goal_y  = static_cast<int>((goal.y  - origin.y) / res);

    // ---- Safety checks ----
    if (!dist_grid.isCellInGrid(start_x, start_y) || !dist_grid.isCellInGrid(goal_x, goal_y)) {
        RCLCPP_WARN(rclcpp::get_logger("greedy_bfs"),
                    "Start or goal out of grid. start=(%d,%d) goal=(%d,%d)",
                    start_x, start_y, goal_x, goal_y);
        return false;
    }

    if (dist_grid.getOccupancy(start_x, start_y) >= 50 ||
        dist_grid.getOccupancy(goal_x, goal_y) >= 50) {
        RCLCPP_WARN(rclcpp::get_logger("greedy_bfs"),
                    "Start or goal in obstacle. start_occ=%d goal_occ=%d",
                    dist_grid.getOccupancy(start_x, start_y),
                    dist_grid.getOccupancy(goal_x, goal_y));
        return false;
    }

    // Greedy Best-First: priority = h(n) only, no g-cost in priority
    std::priority_queue<GreedyNode, std::vector<GreedyNode>, std::greater<>> open;
    std::unordered_map<int, int> came_from;
    std::unordered_set<int> visited;

    const int start_idx = toIndex(start_x, start_y, width);
    const int goal_idx  = toIndex(goal_x, goal_y, width);

    open.push({start_x, start_y, heuristic(start_x, start_y, goal_x, goal_y), 0});
    came_from[start_idx] = start_idx;

    const int dx[8] = {1, -1, 0, 0,  1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1,  1, -1, 1, -1};

    constexpr float min_clearance = 0.18f;  // Increased to keep path farther from walls
    
    int nodes_expanded = 0;

    while (!open.empty()) {
        const GreedyNode current = open.top();
        open.pop();

        const int curr_idx = toIndex(current.x, current.y, width);

        // Skip if already visited
        if (visited.count(curr_idx)) continue;
        visited.insert(curr_idx);
        nodes_expanded++;

        if (curr_idx == goal_idx) {
            // Reconstruct path
            const auto path_indices = reconstructPath(came_from, goal_idx, width);

            for (size_t i = 0; i < path_indices.size(); ++i) {
                const auto& [x, y] = path_indices[i];

                geometry_msgs::msg::Pose2D pose;
                pose.x = x * res + origin.x;
                pose.y = y * res + origin.y;

                if (path_indices.size() >= 2) {
                    if (i + 1 < path_indices.size()) {
                        const int nx = path_indices[i + 1].first;
                        const int ny = path_indices[i + 1].second;
                        pose.theta = std::atan2(static_cast<float>(ny - y),
                                                static_cast<float>(nx - x));
                    } else {
                        pose.theta = goal.theta;
                    }
                } else {
                    pose.theta = goal.theta;
                }

                raw_path.poses.push_back(pose);
            }

            path = smoothPath(dist_grid, raw_path);

            return !path.poses.empty();
        }

        for (int k = 0; k < 8; ++k) {
            const int nx = current.x + dx[k];
            const int ny = current.y + dy[k];

            if (!dist_grid.isCellInGrid(nx, ny)) continue;

            const int neighbor_idx = toIndex(nx, ny, width);
            if (visited.count(neighbor_idx)) continue;

            const int occupancy = dist_grid.getOccupancy(nx, ny);
            if (occupancy >= 50) continue;

            const float obs_dist = dist_grid.getDistance(nx, ny);
            if (obs_dist >= 0.0f && obs_dist < min_clearance) continue;

            // Only add if not yet discovered
            if (!came_from.count(neighbor_idx)) {
                came_from[neighbor_idx] = curr_idx;
                const float h = heuristic(nx, ny, goal_x, goal_y);
                const int new_depth = current.depth + 1;
                // Weighted priority: balance between heuristic and actual depth
                // f = 0.7 * h + 0.3 * depth (favor heuristic but still consider path length)
                const float priority = 0.7f * h + 0.3f * new_depth * 0.1f;  // 0.1f is cell size scaling
                open.push({nx, ny, priority, new_depth});
            }
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("greedy_bfs"),
                "No path found. Expanded %d nodes, open_list had %zu items. start=(%d,%d) goal=(%d,%d) dist=%.2fm",
                nodes_expanded, open.size(), start_x, start_y, goal_x, goal_y,
                std::hypot(goal.x - start.x, goal.y - start.y));
    return false;  // No path found
}

std::vector<std::pair<int, int>> GreedyBestFirstPlanner::reconstructPath(
    const std::unordered_map<int, int>& came_from, int goal_idx, int width) {

    std::vector<std::pair<int, int>> path;

    int current = goal_idx;
    if (!came_from.count(current)) {
        return path;
    }

    while (true) {
        const int x = current % width;
        const int y = current / width;
        path.emplace_back(x, y);

        const int parent = came_from.at(current);
        if (parent == current) break;
        current = parent;

        if (!came_from.count(current)) break;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool GreedyBestFirstPlanner::isLineFree(const ObstacleDistanceGrid& dist_grid,
                                        float x0, float y0, float x1, float y1) {
    const float step = dist_grid.getResolution() * 0.5f;
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float dist = std::hypot(dx, dy);
    const float angle = std::atan2(dy, dx);

    for (float d = 0.0f; d <= dist; d += step) {
        const float x = x0 + std::cos(angle) * d;
        const float y = y0 + std::sin(angle) * d;
        const int ix = static_cast<int>((x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
        const int iy = static_cast<int>((y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());
        if (!dist_grid.isCellInGrid(ix, iy) || dist_grid.getDistance(ix, iy) < 0.18f) {
            return false;
        }
    }
    return true;
}

mbot_interfaces::msg::Pose2DArray GreedyBestFirstPlanner::smoothPath(
    const ObstacleDistanceGrid& dist_grid, const mbot_interfaces::msg::Pose2DArray& raw_path) {

    mbot_interfaces::msg::Pose2DArray pruned;
    if (raw_path.poses.empty()) return pruned;

    pruned.poses.push_back(raw_path.poses.front());

    size_t anchor = 0;
    while (anchor < raw_path.poses.size() - 1) {
        size_t best = anchor + 1;

        for (size_t j = raw_path.poses.size() - 1; j > anchor; --j) {
            const auto& a = raw_path.poses[anchor];
            const auto& b = raw_path.poses[j];
            if (isLineFree(dist_grid, a.x, a.y, b.x, b.y)) {
                best = j;
                break;
            }
        }

        pruned.poses.push_back(raw_path.poses[best]);
        anchor = best;
    }

    if (pruned.poses.size() >= 2) {
        for (size_t i = 0; i + 1 < pruned.poses.size(); ++i) {
            const float ddx = pruned.poses[i + 1].x - pruned.poses[i].x;
            const float ddy = pruned.poses[i + 1].y - pruned.poses[i].y;
            pruned.poses[i].theta = std::atan2(ddy, ddx);
        }
        pruned.poses.back().theta = pruned.poses[pruned.poses.size() - 2].theta;
    }

    return pruned;
}
