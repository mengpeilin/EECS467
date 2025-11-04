#include "mapping.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

OccupancyGrid::OccupancyGrid(float width_m, float height_m, float resolution, float origin_x, float origin_y)
: resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y)
{
    width_ = static_cast<int>(width_m / resolution);
    height_ = static_cast<int>(height_m / resolution);
    data_.resize(width_ * height_, std::numeric_limits<float>::quiet_NaN());
}

void OccupancyGrid::markCellOccupied(int x, int y)
{
    int idx = toIndex(x, y);
    if (idx >= 0 && idx < data_.size())
    {
        if (std::isnan(data_[idx])) data_[idx] = 0.0f;
        // TODO #2: Update log-odds for occupied cell
        // Hints: Utilize LogOdds variables from header file
    }
}

void OccupancyGrid::markCellFree(int x, int y)
{
    int idx = toIndex(x, y);
    if (idx >= 0 && idx < data_.size())
    {
        if (std::isnan(data_[idx])) data_[idx] = 0.0f;
        // TODO #3: Update log-odds for free cell
        // Hints: Utilize LogOdds variables from header file
    }
}

std::vector<int8_t> OccupancyGrid::getOccupancyGrid() const
{
    std::vector<int8_t> result;
    result.reserve(data_.size());

    for (float log_odds : data_)
    {
        if (std::isnan(log_odds)) {
            result.push_back(-1);  // unknown
        } else {
            float prob = 1.0f - 1.0f / (1.0f + std::exp(log_odds));
            int rounded = static_cast<int>(std::round(prob * 100.0f));
            result.push_back(static_cast<int8_t>(std::clamp(rounded, 0, 100)));
        }
    }

    return result;
}

int OccupancyGrid::worldToGridX(float x) const {
    return static_cast<int>((x - origin_x_) / resolution_);
}

int OccupancyGrid::worldToGridY(float y) const {
    return static_cast<int>((y - origin_y_) / resolution_);
}

int OccupancyGrid::toIndex(int x, int y) const
{
    if (x < 0 || y < 0 || x >= width_ || y >= height_)
        return -1;
    return y * width_ + x;
}

std::vector<std::pair<int, int>> bresenhamRayTrace(
    float origin_x, float origin_y, float theta, float range, const OccupancyGrid& grid)
{
    std::vector<std::pair<int, int>> cells;

    // Start and end point in world
    float end_x = origin_x + range * std::cos(theta);
    float end_y = origin_y + range * std::sin(theta);

    // Convert to grid coordinates
    int x0 = grid.worldToGridX(origin_x);
    int y0 = grid.worldToGridY(origin_y);
    int x1 = grid.worldToGridX(end_x);
    int y1 = grid.worldToGridY(end_y);

    // TODO #1: Implement Bresenham's algorithm

    return cells;
}
