#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace mbot_nav;

AStarPlanner::AStarPlanner() {}

float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(static_cast<float>(x2 - x1), static_cast<float>(y2 - y1));
}

bool AStarPlanner::planPath(const ObstacleDistanceGrid& dist_grid,
                            const geometry_msgs::msg::Pose2D& start,
                            const geometry_msgs::msg::Pose2D& goal,
                            mbot_interfaces::msg::Pose2DArray& path) {
    path.poses.clear();
    mbot_interfaces::msg::Pose2DArray raw_path;

    const int width  = dist_grid.getWidth();
    const int height = dist_grid.getHeight();
    (void)height;

    const float res = dist_grid.getResolution();
    const auto& origin = dist_grid.getOrigin().position;

    const int start_x = static_cast<int>((start.x - origin.x) / res);
    const int start_y = static_cast<int>((start.y - origin.y) / res);
    const int goal_x  = static_cast<int>((goal.x  - origin.x) / res);
    const int goal_y  = static_cast<int>((goal.y  - origin.y) / res);

    // ---- Safety checks: start/goal in grid and not in obstacles ----
    if (!dist_grid.isCellInGrid(start_x, start_y) || !dist_grid.isCellInGrid(goal_x, goal_y)) {
        RCLCPP_WARN(rclcpp::get_logger("astar"),
                    "Start or goal out of grid. start=(%d,%d) goal=(%d,%d)",
                    start_x, start_y, goal_x, goal_y);
        return false;
    }

    const int start_occ = dist_grid.getOccupancy(start_x, start_y);
    const int goal_occ  = dist_grid.getOccupancy(goal_x, goal_y);
    if (start_occ >= 50 || goal_occ >= 50) {
        RCLCPP_WARN(rclcpp::get_logger("astar"),
                    "Start or goal in obstacle. start_occ=%d goal_occ=%d", start_occ, goal_occ);
        return false;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> open;
    std::unordered_map<int, float> cost_so_far;
    std::unordered_map<int, int> came_from;

    const int start_idx = toIndex(start_x, start_y, width);
    const int goal_idx  = toIndex(goal_x, goal_y, width);

    open.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y)});
    cost_so_far[start_idx] = 0.0f;
    came_from[start_idx] = start_idx;

    const int dx[8] = {1, -1, 0, 0,  1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1,  1, -1, 1, -1};

    // ---- Tunable cost parameters ----
    constexpr int   OCC_UNKNOWN = -1;
    constexpr float min_clearance = 0.18f;   // meters; robot radius ~17cm + margin

    constexpr float w_unknown = 2.0f;        // penalty for unknown cells (if allowed)
    constexpr float w_clear   = 3.0f;        // penalty for being close to obstacles (increased to push paths away from walls)

    while (!open.empty()) {
        const AStarNode current = open.top();
        open.pop();

        const int curr_idx = toIndex(current.x, current.y, width);
        if (curr_idx == goal_idx) {
            const auto path_indices = reconstructPath(came_from, goal_idx, width);

            for (size_t i = 0; i < path_indices.size(); ++i) {
                const auto& [x, y] = path_indices[i];

                geometry_msgs::msg::Pose2D pose;
                pose.x = x * res + origin.x;
                pose.y = y * res + origin.y;

                // heading: tangent direction along the path
                if (path_indices.size() >= 2) {
                    if (i + 1 < path_indices.size()) {
                        const int nx = path_indices[i + 1].first;
                        const int ny = path_indices[i + 1].second;
                        pose.theta = std::atan2(static_cast<float>(ny - y), static_cast<float>(nx - x));
                    } else {
                        // last waypoint: respect goal heading (if meaningful)
                        pose.theta = goal.theta;
                        // alternative: use last segment direction
                        // const int px = path_indices[i - 1].first;
                        // const int py = path_indices[i - 1].second;
                        // pose.theta = std::atan2(static_cast<float>(y - py), static_cast<float>(x - px));
                    }
                } else {
                    pose.theta = goal.theta;
                }

                raw_path.poses.push_back(pose);
            }

            path = smoothPath(dist_grid, raw_path);   // prune / smooth
            return !path.poses.empty();
        }

        for (int k = 0; k < 8; ++k) {
            const int nx = current.x + dx[k];
            const int ny = current.y + dy[k];

            if (!dist_grid.isCellInGrid(nx, ny)) continue;

            const int neighbor_idx = toIndex(nx, ny, width);
            const float obs_dist = dist_grid.getDistance(nx, ny);
            const int occupancy = dist_grid.getOccupancy(nx, ny);

            // Block known obstacles
            if (occupancy >= 50) continue;

            // Keep clearance from obstacles using distance field
            if (obs_dist >= 0.0f && obs_dist < min_clearance) continue;

            float penalty = 0.0f;

            // Unknown: mild penalty (allowing exploration-style paths)
            if (occupancy == OCC_UNKNOWN) {
                penalty += w_unknown;
            }

            // Prefer larger clearance: inverse-distance penalty
            if (obs_dist > 0.0f) {
                penalty += w_clear / (obs_dist + 0.10f);
            } else {
                // distance invalid -> treat similarly to unknown
                penalty += w_unknown;
            }

            // Move cost: 1 for 4-neighbor, sqrt(2) for diagonal
            const float move_cost = std::hypot(static_cast<float>(dx[k]), static_cast<float>(dy[k]));

            const float new_cost = cost_so_far[curr_idx] + move_cost + penalty;

            if (!cost_so_far.count(neighbor_idx) || new_cost < cost_so_far[neighbor_idx]) {
                cost_so_far[neighbor_idx] = new_cost;
                came_from[neighbor_idx] = curr_idx;

                const float h = heuristic(nx, ny, goal_x, goal_y);
                open.push({nx, ny, new_cost, h});
            }
        }
    }

    return false;  // Path not found
}

std::vector<std::pair<int, int>> AStarPlanner::reconstructPath(
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

        if (!came_from.count(current)) break;  // safety
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool AStarPlanner::isLineFree(const ObstacleDistanceGrid& dist_grid, float x0, float y0, float x1, float y1)
{
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

mbot_interfaces::msg::Pose2DArray AStarPlanner::smoothPath(
    const ObstacleDistanceGrid& dist_grid, const mbot_interfaces::msg::Pose2DArray& raw_path)
{
    mbot_interfaces::msg::Pose2DArray pruned;
    if (raw_path.poses.empty()) return pruned;

    pruned.poses.push_back(raw_path.poses.front());

    // Greedy shortcutting: keep farthest reachable waypoint each time
    size_t anchor = 0;
    while (anchor < raw_path.poses.size() - 1) {
        size_t best = anchor + 1;

        // Try to jump as far as possible while line is free
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

    // Recompute headings after pruning
    if (pruned.poses.size() >= 2) {
        for (size_t i = 0; i + 1 < pruned.poses.size(); ++i) {
            const float dx = pruned.poses[i + 1].x - pruned.poses[i].x;
            const float dy = pruned.poses[i + 1].y - pruned.poses[i].y;
            pruned.poses[i].theta = std::atan2(dy, dx);
        }
        pruned.poses.back().theta = pruned.poses[pruned.poses.size() - 2].theta;
    }

    return pruned;
}