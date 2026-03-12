#include "particle_filter.hpp"
#include "slam_utils.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace mbot_slam;


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
    RCLCPP_INFO(rclcpp::get_logger("particle_filter"), 
                "uniform weight initialization %f", w);
    for (auto& p : particle_cloud_.particles) {
        p.pose   = pose;
        p.weight = w;
    }
    pose_estimate_ = pose;
}

// Main update cycle
// geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&     odom,
//                                                 const sensor_msgs::msg::LaserScan& scan,
//                                                 const ObstacleDistanceGrid&        dist_grid)
// {
//     bool moved = action_model_.processOdometry(odom);
//     nav2_msgs::msg::ParticleCloud resampled_particles = systematicResample();
//     nav2_msgs::msg::ParticleCloud prior = moved ? propagate(resampled_particles) : resampled_particles;
//     particle_cloud_ = weightParticles(prior, scan, dist_grid);
//     pose_estimate_ = computeBestEstimate(particle_cloud_);

//     return pose_estimate_;
// }

geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&     odom,
                                                const sensor_msgs::msg::LaserScan& scan,
                                                const ObstacleDistanceGrid&        dist_grid)
{
    bool moved = action_model_.processOdometry(odom);
    nav2_msgs::msg::ParticleCloud prior = moved ? propagate(particle_cloud_) : particle_cloud_;
    particle_cloud_ = weightParticles(prior, scan, dist_grid);

    // double neff = computeEffectiveSampleSize(particle_cloud_);
    // if (neff < RESAMPLE_THRESHOLD * num_particles_) {
    //     particle_cloud_ = systematicResample();
    // }
    particle_cloud_ = systematicResample();
    pose_estimate_ = computeBestEstimate(particle_cloud_);

    return pose_estimate_;
}


// Systematic resampling: keep high-weight particles, discard low-weight ones
nav2_msgs::msg::ParticleCloud ParticleFilter::systematicResample() const
{

    // TODO #2: Resample the particles
    // Step 1: Build cumulative distribution of weights
    std::vector<double> cdf(num_particles_);
    cdf[0] = particle_cloud_.particles[0].weight;
    for (int k = 1; k < num_particles_; ++k)
        cdf[k] = cdf[k-1] + particle_cloud_.particles[k].weight;

    // Step 2: Handle edge case (all particles have zero weight)
    const double total_w = cdf.back();
    if (total_w <= 0.0) {
        return particle_cloud_;  // No resampling possible
    }

    // Step 3: Normalize to probability [0, 1]
    for (double& v : cdf) v /= total_w;

    // Step 4: Draw N equally-spaced samples and copy particles
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);

    std::uniform_real_distribution<double> unif(0.0, 1.0 / num_particles_);
    double random_offset = unif(random_gen);
    int particle_idx = 0;

    for (int sample = 0; sample < num_particles_; ++sample) {
        double sample_threshold = random_offset + static_cast<double>(sample) / num_particles_;
        while (sample_threshold > cdf[particle_idx]) particle_idx++;

        resampled.particles.push_back(particle_cloud_.particles[particle_idx]);
        resampled.particles.back().weight = 1.0 / num_particles_;  // uniform weight
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
                                                                const sensor_msgs::msg::LaserScan& scan,
                                                                const ObstacleDistanceGrid& dist_grid) const
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

geometry_msgs::msg::Pose ParticleFilter::computeBestEstimate(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    const double TOP_PARTICLE_FRACTION = 0.1;  // Use top x% of particles

    // Create (weight, index) pairs and sort by weight (descending)
    std::vector<std::pair<double, int>> weight_indices;
    weight_indices.reserve(num_particles_);
    for (int i = 0; i < num_particles_; ++i) {
        weight_indices.push_back({cloud.particles[i].weight, i});
    }
    std::sort(weight_indices.begin(), weight_indices.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    int num_top = std::max(1, static_cast<int>(num_particles_ * TOP_PARTICLE_FRACTION));

    double x=0.0, y=0.0, cos_sum=0.0, sin_sum=0.0;
    double sum_w = 0.0;

    // Compute weighted average using only top particles
    for (int i = 0; i < num_top; ++i) {
        int idx = weight_indices[i].second;
        const auto& p = cloud.particles[idx];
        const double w = p.weight;
        const double th = yawFromQuaternion(p.pose.orientation);

        x += w * p.pose.position.x;
        y += w * p.pose.position.y;
        cos_sum += w * std::cos(th);
        sin_sum += w * std::sin(th);
        sum_w += w;
    }

    // Normalize by sum of top particle weights
    if (sum_w > 0.0) {
        x /= sum_w;
        y /= sum_w;
        cos_sum /= sum_w;
        sin_sum /= sum_w;
    }

    geometry_msgs::msg::Pose est;
    est.position.x = x;
    est.position.y = y;
    est.position.z = 0.0;
    setOrientationFromYaw(est, std::atan2(sin_sum, cos_sum));
    return est;
}

// Compute Effective Particle Number for adaptive resampling
// Neff = 1 / sum(w_i^2)
double ParticleFilter::computeEffectiveSampleSize(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    double sum_w_squared = 0.0;
    for (const auto& p : cloud.particles) {
        sum_w_squared += p.weight * p.weight;
    }

    // Avoid division by zero
    if (sum_w_squared <= 0.0) {
        return 1.0;
    }

    return 1.0 / sum_w_squared;
}