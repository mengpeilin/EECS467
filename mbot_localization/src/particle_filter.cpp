#include "particle_filter.hpp"
#include "localization_utils.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include <vector>
#include <utility>
// add time computing
#include <chrono>
#include <iostream>


using namespace mbot_localization;


// Constructor
ParticleFilter::ParticleFilter()
: num_particles_(NUM_PARTICLES),
  action_model_(),
  sensor_model_()
{
    particle_cloud_.particles.resize(num_particles_);

    std::random_device seed_source;
    random_gen = std::mt19937(seed_source());
}

void ParticleFilter::initializeAtPose(const geometry_msgs::msg::Pose& pose)
{
    const double w = 1.0 / static_cast<double>(num_particles_);
    for (auto& p : particle_cloud_.particles) {
        p.pose   = pose;
        p.weight = w;
    }
    pose_estimate_ = pose;
}

void ParticleFilter::initializeGlobal(const nav_msgs::msg::OccupancyGrid& map)
{
    // Step 1: Identify all free cells (occupancy == 0 means free space)
    std::vector<std::pair<int,int>> free_cells;
    const int w = static_cast<int>(map.info.width);
    const int h = static_cast<int>(map.info.height);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int8_t val = map.data[y * w + x];
            if (val == 0) {  // free space
                free_cells.emplace_back(x, y);
            }
        }
    }

    if (free_cells.empty()) {
        std::cerr << "[PF] initializeGlobal: no free cells found in map!" << std::endl;
        return;
    }

    // Step 2: Randomly sample particle positions from free cells
    std::uniform_int_distribution<size_t> cell_dist(0, free_cells.size() - 1);
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
    // Small offset within cell to avoid all particles landing on cell centers
    std::uniform_real_distribution<double> offset_dist(0.0, 1.0);

    const double res = map.info.resolution;
    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;

    const double equal_w = 1.0 / static_cast<double>(num_particles_);

    particle_cloud_.particles.resize(num_particles_);
    for (auto& p : particle_cloud_.particles) {
        const auto& cell = free_cells[cell_dist(random_gen)];
        // Convert grid cell to world coordinates with random sub-cell offset
        p.pose.position.x = ox + (cell.first + offset_dist(random_gen)) * res;
        p.pose.position.y = oy + (cell.second + offset_dist(random_gen)) * res;
        p.pose.position.z = 0.0;
        // Step 3: Assign random orientation in [-pi, pi]
        const double yaw = angle_dist(random_gen);
        mbot_localization::setOrientationFromYaw(p.pose, yaw);
        // Step 4: Equal weights
        p.weight = equal_w;
    }

    pose_estimate_ = computeBestEstimate(particle_cloud_);
    std::cout << "[PF] Global init: " << num_particles_ << " particles across "
              << free_cells.size() << " free cells" << std::endl;
}

double ParticleFilter::computeVariance() const
{
    if (particle_cloud_.particles.empty()) return 1e9;

    double mean_x = 0.0, mean_y = 0.0;
    for (const auto& p : particle_cloud_.particles) {
        mean_x += p.weight * p.pose.position.x;
        mean_y += p.weight * p.pose.position.y;
    }

    double var_x = 0.0, var_y = 0.0;
    for (const auto& p : particle_cloud_.particles) {
        const double dx = p.pose.position.x - mean_x;
        const double dy = p.pose.position.y - mean_y;
        var_x += p.weight * dx * dx;
        var_y += p.weight * dy * dy;
    }

    return var_x + var_y;
}

// Main update cycle
geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&      odom,
                                                const sensor_msgs::msg::LaserScan& scan,
                                                const ObstacleDistanceGrid&        dist_grid)
{
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();

    bool moved = action_model_.processOdometry(odom);
    nav2_msgs::msg::ParticleCloud resampled_particles = systematicResample();
    nav2_msgs::msg::ParticleCloud prior = moved ? propagate(resampled_particles) : resampled_particles;
    particle_cloud_ = weightParticles(prior, scan, dist_grid);
    pose_estimate_ = computeBestEstimate(particle_cloud_);

    const auto t1 = clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    static int cnt = 0;
    static double sum_ms = 0.0;
    cnt++;
    sum_ms += ms;

    // 每 100 次打印一次平均耗时，避免刷屏
    if (cnt % 100 == 0) {
        std::cout << "[PF] avg update time over last 100: " << (sum_ms / 100.0)
                  << " ms, particles=" << num_particles_ << std::endl;
        sum_ms = 0.0;
    }

    return pose_estimate_;
}


// Systematic resampling: keep high-weight particles, discard low-weight ones
nav2_msgs::msg::ParticleCloud ParticleFilter::systematicResample() const
{
    // Step 4 in skeleton creates resampled; keep that structure
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);

    if (particle_cloud_.particles.empty()) return resampled;

    // TODO #2: Resample the particles
    // Step 1: Build cumulative distribution of weights
    std::vector<double> cdf(particle_cloud_.particles.size());
    double cum = 0.0;
    for (size_t i = 0; i < particle_cloud_.particles.size(); ++i) {
        cum += particle_cloud_.particles[i].weight;
        cdf[i] = cum;
    }

    // Step 2: Handle edge case (if all particles have zero weight?)
    if (!(cum > 0.0) || !std::isfinite(cum)) {
        const double w = 1.0 / static_cast<double>(num_particles_);
        for (int i = 0; i < num_particles_; ++i) {
            nav2_msgs::msg::Particle p = particle_cloud_.particles[i % particle_cloud_.particles.size()];
            p.weight = w;
            resampled.particles.push_back(p);
        }
        return resampled;
    }

    // Step 3: Normalize to probability [0, 1]
    for (auto& v : cdf) v /= cum;

    // Step 4: Draw N equally-spaced samples and copy particles.
    // You may find std::uniform_real_distribution<double> unif() helpful.
    std::uniform_real_distribution<double> unif(0.0, 1.0 / static_cast<double>(num_particles_));
    const double r = unif(random_gen);
    const double step = 1.0 / static_cast<double>(num_particles_);

    size_t idx = 0;
    for (int m = 0; m < num_particles_; ++m) {
        const double u = r + m * step;
        while (idx + 1 < cdf.size() && u > cdf[idx]) ++idx;

        nav2_msgs::msg::Particle p = particle_cloud_.particles[idx];
        p.weight = 1.0 / static_cast<double>(num_particles_);
        resampled.particles.push_back(p);
    }

    return resampled;
}

// Propagate each particle through ActionModel
nav2_msgs::msg::ParticleCloud ParticleFilter::propagate(const nav2_msgs::msg::ParticleCloud& resampled_particles)
{
    nav2_msgs::msg::ParticleCloud prior;
    prior.particles.reserve(num_particles_);

    for (const auto& particle : resampled_particles.particles) {
        nav2_msgs::msg::Particle q = particle;
        q.pose = action_model_.propagateParticle(particle.pose);
        prior.particles.push_back(std::move(q));
    }
    return prior;
}

// Weight each particle with SensorModel
nav2_msgs::msg::ParticleCloud ParticleFilter::weightParticles(const nav2_msgs::msg::ParticleCloud& prior,
                                                              const sensor_msgs::msg::LaserScan&  scan,
                                                              const ObstacleDistanceGrid&         dist_grid) const
{
    nav2_msgs::msg::ParticleCloud posterior = prior;
    double sum_w = 0.0;

    for (auto& p : posterior.particles) {
        p.weight = sensor_model_.likelihood(p.pose, scan, dist_grid);
        sum_w   += p.weight;
    }

    // Avoid division by zero
    if (sum_w <= 0.0) {
        const double w = 1.0 / static_cast<double>(num_particles_);
        for (auto& p : posterior.particles) p.weight = w;
        return posterior;
    }

    // Normalize
    for (auto& p : posterior.particles) p.weight /= sum_w;
    return posterior;
}

// Compute pose estimate (weighted mean on SE2)
geometry_msgs::msg::Pose ParticleFilter::computeBestEstimate(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    double x=0.0, y=0.0, cos_sum=0.0, sin_sum=0.0;

    // TODO #5: Compute the best estimate pose
    // Loop through all p in cloud.particles. Fill in x, y, cos_sum, sin_sum
    for (const auto& p : cloud.particles) {
        const double w = p.weight;
        const double yaw = yawFromQuaternion(p.pose.orientation);
        x += w * p.pose.position.x;
        y += w * p.pose.position.y;
        cos_sum += w * std::cos(yaw);
        sin_sum += w * std::sin(yaw);
    }

    geometry_msgs::msg::Pose est;
    est.position.x = x;
    est.position.y = y;
    est.position.z = 0.0;
    setOrientationFromYaw(est, std::atan2(sin_sum, cos_sum));
    return est;
}
