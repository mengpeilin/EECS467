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
    std::queue<GridCell> queue;

    // Compute distances to nearest obstacle for each cell, then fill distances_ vector
    // occupancy_ is 1D array representing 2D grid
    // distances_ is also 1D array representing 2D grid
    // later we will use dist_grid_ in sensor model to compare expected vs actual measurements

    // step 1 - Initialize queue with all occupied cells
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            if (occupancy_[idx] >= 50) {
                distances_[idx] = 0.0f;
                queue.emplace(x, y);
            }
        }
    }

    // Step 2 TODO: Flood-fill using 8-connected neighbors and Euclidean distance
    //  Compute the neighbors, loop through the queue, update neighbor nx, ny,  then queue.emplace(nx, ny)
    //  You are also encouraged to explore other approaches to finish the distance computation.
    // Step 2: Flood-fill using 8-connected neighbors and Euclidean distance
    // 8-connected neighbor offsets
    const int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
    const int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };

    while (!queue.empty()) {
        const GridCell cur = queue.front();
        queue.pop();

        const int cx = cur.x;
        const int cy = cur.y;
        const int cidx = cy * width_ + cx;
        const float cur_dist = distances_[cidx];

        // Try to relax all 8 neighbors
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dx[k];
            const int ny = cy + dy[k];
            if (!isCellInGrid(nx, ny)) continue;

            const int nidx = ny * width_ + nx;

            // Step cost: straight = res, diagonal = res*sqrt(2)
            const bool diagonal = (dx[k] != 0 && dy[k] != 0);
            const float step = diagonal ? (resolution_ * std::sqrt(2.0f)) : resolution_;

            const float new_dist = cur_dist + step;

            // If we found a shorter path to obstacle, update and push
            if (new_dist < distances_[nidx]) {
                distances_[nidx] = new_dist;
                queue.emplace(nx, ny);
            }
        }
    }


   
    


}

