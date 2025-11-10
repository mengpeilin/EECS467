#include "moving_laser_scan.hpp"
#include "slam_utils.hpp"
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Linear interpolation for geometry_msgs::msg::Point
inline geometry_msgs::msg::Point linearInterpolatePoint(const geometry_msgs::msg::Point& a,
                                           const geometry_msgs::msg::Point& b,
                                           float t)
{
    geometry_msgs::msg::Point out;
    // TODO #2: Linear interpolation along x, y, z
    return out;
}


MovingLaserScan::MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                                 const Pose& start_pose,
                                 const Pose& end_pose)
{
    interpolateRay(scan, start_pose, end_pose);
}

void MovingLaserScan::interpolateRay(const sensor_msgs::msg::LaserScan& scan,
                                     const Pose& start_pose,
                                     const Pose& end_pose)
{
    const size_t num_rays = scan.ranges.size();
    rays_.clear();
    rays_.reserve(num_rays);

    // Extract yaw from quaternions (world-frame base yaw at scan start/end)
    const double yaw_start = mbot_slam::yawFromQuaternion(start_pose.orientation);
    const double yaw_end = mbot_slam::yawFromQuaternion(end_pose.orientation);

    // Shortest angular delta
    double delta_yaw = mbot_slam::wrapToPi(yaw_end - yaw_start);

    // Iterate over all beams
    for (size_t i = 0; i < num_rays; ++i)
    {
        const float range = scan.ranges[i];

        // Drop garbage / under-range / out-of-range
        if (std::isnan(range) || range < scan.range_min || range >= scan.range_max) {
            continue;
        }

        // TODO #1: Fill up linearInterpolatePoint to get origin
        // TODO #3: Calculate beam's interpolated angle

        InterpolatedRay ray;
        // TODO #4: Fill in the ray fields
        rays_.push_back(ray);
    }
}
