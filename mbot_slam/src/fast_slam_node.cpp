/**
 * Fast SLAM Node for Maze Speed Challenge
 * 
 * Optimized for speed over accuracy:
 * - Fewer particles (200 instead of 800)
 * - Larger ray stride (process fewer laser beams)
 * - Less frequent map publishing
 * - Coarser grid resolution (0.03m instead of 0.02m)
 * - Faster TF publishing
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "particle_filter.hpp"
#include "slam_utils.hpp"
#include "mapping.hpp"
#include "moving_laser_scan.hpp"
#include "obstacle_distance_grid.hpp"

#include <algorithm>
#include <tf2/utils.h>
#include <cmath>
#include <mutex>
#include <random>

// Fast Particle Filter with fewer particles
class FastParticleFilter
{
public:
    static constexpr int NUM_PARTICLES = 200;  // Reduced from 800
    static constexpr double SIGMA_HIT = 0.2;   // Slightly more tolerant
    static constexpr int RAY_STRIDE = 4;       // Process every 4th ray (faster)
    static constexpr double Z_HIT = 0.9;
    static constexpr double Z_RAND = 0.1;

    FastParticleFilter()
    {
        std::random_device rd;
        random_gen_ = std::mt19937(rd());
        particles_.resize(NUM_PARTICLES);
    }

    void initializeAtPose(const geometry_msgs::msg::Pose& pose)
    {
        const double w = 1.0 / static_cast<double>(NUM_PARTICLES);
        for (auto& p : particles_) {
            p.pose = pose;
            p.weight = w;
        }
        pose_estimate_ = pose;
    }

    void resetOdometry(const nav_msgs::msg::Odometry& odom)
    {
        prev_odom_ = odom;
    }

    geometry_msgs::msg::Pose update(const nav_msgs::msg::Odometry& odom,
                                     const sensor_msgs::msg::LaserScan& scan,
                                     const ObstacleDistanceGrid& dist_grid)
    {
        // Simple odometry-based motion model
        bool moved = processOdometry(odom);
        
        if (moved) {
            propagateParticles();
        }

        // Weight particles
        weightParticles(scan, dist_grid);

        // Resample
        resample();

        // Compute estimate
        pose_estimate_ = computeWeightedMean();
        return pose_estimate_;
    }

    const geometry_msgs::msg::Pose& getPoseEstimate() const { return pose_estimate_; }

private:
    struct Particle {
        geometry_msgs::msg::Pose pose;
        double weight = 1.0;
    };

    std::vector<Particle> particles_;
    geometry_msgs::msg::Pose pose_estimate_;
    nav_msgs::msg::Odometry prev_odom_;
    mutable std::mt19937 random_gen_;

    // Motion deltas
    double rot1_ = 0.0, trans_ = 0.0, rot2_ = 0.0;
    double rot1_std_ = 0.01, trans_std_ = 0.01, rot2_std_ = 0.01;

    bool processOdometry(const nav_msgs::msg::Odometry& odom)
    {
        double dx = odom.pose.pose.position.x - prev_odom_.pose.pose.position.x;
        double dy = odom.pose.pose.position.y - prev_odom_.pose.pose.position.y;
        
        double theta_prev = yawFromQuaternion(prev_odom_.pose.pose.orientation);
        double theta_curr = yawFromQuaternion(odom.pose.pose.orientation);

        double delta_trans = std::sqrt(dx*dx + dy*dy);
        double delta_theta = wrapToPi(theta_curr - theta_prev);

        bool moved = (delta_trans >= 0.01) || (std::abs(delta_theta) >= 0.02);

        if (moved) {
            rot1_ = wrapToPi(std::atan2(dy, dx) - theta_prev);
            trans_ = delta_trans;
            rot2_ = wrapToPi(delta_theta - rot1_);

            // Noise model (aggressive for speed)
            double k1 = 0.3, k2 = 0.2;
            rot1_std_ = std::sqrt(k1 * std::max(std::abs(rot1_), 0.02));
            trans_std_ = std::sqrt(k2 * std::max(std::abs(trans_), 0.01));
            rot2_std_ = std::sqrt(k1 * std::max(std::abs(rot2_), 0.02));
        }

        prev_odom_ = odom;
        return moved;
    }

    void propagateParticles()
    {
        std::normal_distribution<double> n_rot1(rot1_, rot1_std_);
        std::normal_distribution<double> n_trans(trans_, trans_std_);
        std::normal_distribution<double> n_rot2(rot2_, rot2_std_);

        for (auto& p : particles_) {
            double sr1 = n_rot1(random_gen_);
            double st = n_trans(random_gen_);
            double sr2 = n_rot2(random_gen_);

            double theta = yawFromQuaternion(p.pose.orientation);
            p.pose.position.x += st * std::cos(theta + sr1);
            p.pose.position.y += st * std::sin(theta + sr1);
            setOrientationFromYaw(p.pose, wrapToPi(theta + sr1 + sr2));
        }
    }

    void weightParticles(const sensor_msgs::msg::LaserScan& scan,
                         const ObstacleDistanceGrid& dist_grid)
    {
        double sum_w = 0.0;
        const double inv_two_sigma2 = 1.0 / (2.0 * SIGMA_HIT * SIGMA_HIT);
        const double p_uniform = 1.0 / std::max(1e-3, static_cast<double>(scan.range_max - scan.range_min));

        for (auto& p : particles_) {
            double log_sum = 0.0;
            int used = 0;

            double robot_x = p.pose.position.x;
            double robot_y = p.pose.position.y;
            double robot_yaw = yawFromQuaternion(p.pose.orientation);

            for (size_t i = 0; i < scan.ranges.size(); i += RAY_STRIDE) {
                double range = scan.ranges[i];
                if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max)
                    continue;

                double scan_angle = scan.angle_min + i * scan.angle_increment;
                double world_angle = wrapToPi(scan_angle + robot_yaw + M_PI);

                double endpoint_x = robot_x + range * std::cos(world_angle);
                double endpoint_y = robot_y + range * std::sin(world_angle);

                float dist;
                if (!getDistanceAt(endpoint_x, endpoint_y, dist_grid, dist))
                    continue;

                double d_clamped = std::min<double>(dist, 3.0 * SIGMA_HIT);
                double p_hit = std::exp(-(d_clamped * d_clamped) * inv_two_sigma2);
                double prob = std::max(Z_HIT * p_hit + Z_RAND * p_uniform, 1e-12);
                log_sum += std::log(prob);
                ++used;
            }

            p.weight = (used > 0) ? std::max(std::exp(log_sum / used), 1e-9) : 1e-9;
            sum_w += p.weight;
        }

        // Normalize
        if (sum_w > 0.0) {
            for (auto& p : particles_) p.weight /= sum_w;
        } else {
            double w = 1.0 / NUM_PARTICLES;
            for (auto& p : particles_) p.weight = w;
        }
    }

    bool getDistanceAt(double x, double y, const ObstacleDistanceGrid& grid, float& dist) const
    {
        const auto& origin = grid.getOrigin();
        float res = grid.getResolution();
        int gx = static_cast<int>((x - origin.position.x) / res);
        int gy = static_cast<int>((y - origin.position.y) / res);

        if (!grid.isCellInGrid(gx, gy)) return false;
        dist = grid.getDistance(gx, gy);
        return true;
    }

    void resample()
    {
        // Low-variance resampling
        std::vector<double> cdf(NUM_PARTICLES);
        cdf[0] = particles_[0].weight;
        for (int i = 1; i < NUM_PARTICLES; ++i)
            cdf[i] = cdf[i-1] + particles_[i].weight;

        double total_w = cdf.back();
        if (total_w <= 0) return;

        for (auto& v : cdf) v /= total_w;

        std::vector<Particle> resampled;
        resampled.reserve(NUM_PARTICLES);

        std::uniform_real_distribution<double> unif(0.0, 1.0 / NUM_PARTICLES);
        double r = unif(random_gen_);
        int idx = 0;

        for (int i = 0; i < NUM_PARTICLES; ++i) {
            double u = r + static_cast<double>(i) / NUM_PARTICLES;
            while (u > cdf[idx]) ++idx;
            resampled.push_back(particles_[idx]);
            resampled.back().weight = 1.0 / NUM_PARTICLES;
        }

        particles_ = std::move(resampled);
    }

    geometry_msgs::msg::Pose computeWeightedMean() const
    {
        double x = 0, y = 0, cos_sum = 0, sin_sum = 0;
        double sum_w = 0;

        for (const auto& p : particles_) {
            double theta = yawFromQuaternion(p.pose.orientation);
            x += p.weight * p.pose.position.x;
            y += p.weight * p.pose.position.y;
            cos_sum += p.weight * std::cos(theta);
            sin_sum += p.weight * std::sin(theta);
            sum_w += p.weight;
        }

        geometry_msgs::msg::Pose est;
        if (sum_w > 0) {
            est.position.x = x / sum_w;
            est.position.y = y / sum_w;
            setOrientationFromYaw(est, std::atan2(sin_sum / sum_w, cos_sum / sum_w));
        }
        return est;
    }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny, cosy);
    }

    static void setOrientationFromYaw(geometry_msgs::msg::Pose& pose, double yaw)
    {
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(yaw / 2.0);
        pose.orientation.w = std::cos(yaw / 2.0);
    }

    static double wrapToPi(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};


class FastSLAMNode final : public rclcpp::Node
{
public:
    FastSLAMNode()
    : Node("fast_slam_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      grid_(10.0, 10.0, 0.03, -5.0, -5.0)  // Coarser resolution: 0.03m
    {
        // Fast particle filter
        particle_filter_ = std::make_unique<FastParticleFilter>();

        // Transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

        // Map timer: less frequent (750ms instead of 500ms)
        map_timer_ = create_wall_timer(std::chrono::milliseconds(750),
                                       std::bind(&FastSLAMNode::publishMap, this));

        // TF timer: faster (30ms = ~33Hz)
        tf_timer_ = create_wall_timer(std::chrono::milliseconds(30),
                                      std::bind(&FastSLAMNode::publishTfTimer, this));

        // Publish identity map->odom TF immediately so 'map' frame exists from the start.
        // Without this, exploration and motion_controller fail with "map frame does not exist"
        // until the first laser scan is processed.
        latest_tf_map_odom_.setIdentity();
        has_tf_data_ = true;
        publishTfTimer();

        // Subscriptions
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&FastSLAMNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Fast SLAM node initialized (200 particles, ray_stride=4, res=0.03m)");
    }

private:
    std::unique_ptr<FastParticleFilter> particle_filter_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    mbot_slam::OccupancyGrid grid_;
    ObstacleDistanceGrid dist_grid_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    std::mutex tf_mutex_;
    tf2::Transform latest_tf_map_odom_;
    bool has_tf_data_{false};
    bool has_initialized_{false};

    int scan_count_ = 0;
    static constexpr int SCAN_SUBSAMPLE = 2;  // Process every 2nd scan

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Subsample scans for speed
        scan_count_++;
        if (scan_count_ % SCAN_SUBSAMPLE != 0) return;

        // Lookup latest odometry (use TimePointZero for robustness at high speed;
        // exact scan timestamp lookup can fail if TF buffer hasn't caught up yet)
        nav_msgs::msg::Odometry odom_msg;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_base =
                tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

            odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
            odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
            odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
            odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                  "Odom transform failed: %s", ex.what());
            return;
        }

        if (!has_initialized_) {
            particle_filter_->initializeAtPose(odom_msg.pose.pose);
            particle_filter_->resetOdometry(odom_msg);

            MovingLaserScan initial_scan(*scan, odom_msg.pose.pose, odom_msg.pose.pose);
            updateGridWithScan(initial_scan);

            has_initialized_ = true;

            // Broadcast TF from first scan: map==odom at start (identity correction)
            broadcastTf(odom_msg.pose.pose, odom_msg.pose.pose);
            RCLCPP_INFO(get_logger(), "Fast SLAM initialized at odom origin");
            return;
        }

        // Update particle filter
        auto est_pose = particle_filter_->update(odom_msg, *scan, dist_grid_);

        // Update map (use est_pose for both start and end for simplicity/speed)
        MovingLaserScan moving_scan(*scan, est_pose, est_pose);
        updateGridWithScan(moving_scan);

        // Broadcast TF
        broadcastTf(est_pose, odom_msg.pose.pose);

        // Publish pose
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose = est_pose;
        pose_pub_->publish(pose_msg);
    }

    void updateGridWithScan(const MovingLaserScan& scan)
    {
        int ray_count = 0;
        for (const auto& ray : scan) {
            // Process every 2nd ray for speed
            if (++ray_count % 2 != 0) continue;

            auto ray_cells = bresenhamRayTrace(
                ray.origin.x, ray.origin.y, ray.theta, ray.range, grid_);

            if (!ray_cells.empty()) {
                for (size_t j = 0; j + 1 < ray_cells.size(); ++j) {
                    grid_.markCellFree(ray_cells[j].first, ray_cells[j].second);
                }
                grid_.markCellOccupied(ray_cells.back().first, ray_cells.back().second);
            }
        }

        dist_grid_.computeDistFromMap(buildMapMessage());
    }

    nav_msgs::msg::OccupancyGrid buildMapMessage()
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();
        map_msg.info.resolution = grid_.getResolution();
        map_msg.info.width = grid_.getWidth();
        map_msg.info.height = grid_.getHeight();
        map_msg.info.origin.position.x = grid_.getOriginX();
        map_msg.info.origin.position.y = grid_.getOriginY();
        map_msg.info.origin.orientation.w = 1.0;
        map_msg.data = grid_.getOccupancyGrid();
        return map_msg;
    }

    void publishMap()
    {
        map_pub_->publish(buildMapMessage());
    }

    void broadcastTf(const geometry_msgs::msg::Pose& map_base,
                     const geometry_msgs::msg::Pose& odom_base)
    {
        tf2::Transform tf_map_base, tf_odom_base;
        tf2::fromMsg(map_base, tf_map_base);
        tf2::fromMsg(odom_base, tf_odom_base);

        tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();

        {
            std::lock_guard<std::mutex> lock(tf_mutex_);
            latest_tf_map_odom_ = tf_map_odom;
            has_tf_data_ = true;
        }

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = this->now();
        out.header.frame_id = "map";
        out.child_frame_id = "odom";
        out.transform = tf2::toMsg(tf_map_odom);
        tf_broadcaster_->sendTransform(out);
    }

    void publishTfTimer()
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        if (!has_tf_data_) return;

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = this->now();
        out.header.frame_id = "map";
        out.child_frame_id = "odom";
        out.transform = tf2::toMsg(latest_tf_map_odom_);
        tf_broadcaster_->sendTransform(out);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastSLAMNode>());
    rclcpp::shutdown();
    return 0;
}
