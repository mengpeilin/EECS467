#include "obstacle_distance_grid.hpp"
#include <limits>
#include <cmath>

ObstacleDistanceGrid::ObstacleDistanceGrid()
: width_(0), height_(0), resolution_(0.05f)
{}

void ObstacleDistanceGrid::computeDistFromMap(const nav_msgs::msg::OccupancyGrid &map)
{
    width_ = map.info.width;
    height_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_ = map.info.origin;

    occupancy_ = map.data;
    distances_.assign(width_ * height_, std::numeric_limits<float>::max());

    computeDistances();
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

float ObstacleDistanceGrid::getDistance(int x, int y) const
{
    if (!isCellInGrid(x, y)) return std::numeric_limits<float>::max();
    return distances_[y * width_ + x];
}

int8_t ObstacleDistanceGrid::getOccupancy(int x, int y) const
{
    if (!isCellInGrid(x, y)) return -1;  // unknown if out of bounds
    return occupancy_[y * width_ + x];
}

void ObstacleDistanceGrid::computeDistances()
{
    // Use multi-source Dijkstra to compute approximate Euclidean distance on 8-connected grid
    struct PQNode {
        int x;
        int y;
        float d;
        bool operator>(const PQNode& other) const { return d > other.d; }
    };

    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;

    // Step 1: Initialize with all occupied cells (value >= 50)
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const int idx = y * width_ + x;
            if (occupancy_[idx] >= 50) {
                distances_[idx] = 0.0f;
                pq.push({x, y, 0.0f});
            } else {
                distances_[idx] = std::numeric_limits<float>::max();
            }
        }
    }

    // If no obstacle exists in the map, keep all distances as max
    if (pq.empty()) return;

    const int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1};
    const int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1};

    const float step_straight = resolution_;
    const float step_diag = resolution_ * std::sqrt(2.0f);

    // Step 2: Dijkstra relaxations over 8-connected neighbors
    while (!pq.empty()) {
        PQNode cur = pq.top();
        pq.pop();

        const int cur_idx = cur.y * width_ + cur.x;

        // stale entry check
        if (cur.d > distances_[cur_idx]) continue;

        for (int k = 0; k < 8; ++k) {
            const int nx = cur.x + dx[k];
            const int ny = cur.y + dy[k];
            if (!isCellInGrid(nx, ny)) continue;

            const int nidx = ny * width_ + nx;

            const bool is_diag = (dx[k] != 0 && dy[k] != 0);
            const float step = is_diag ? step_diag : step_straight;

            const float nd = cur.d + step;
            if (nd < distances_[nidx]) {
                distances_[nidx] = nd;
                pq.push({nx, ny, nd});
            }
        }
    }
}
