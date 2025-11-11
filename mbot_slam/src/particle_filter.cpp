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
geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&     odom,
                                                const sensor_msgs::msg::LaserScan& scan,
                                                const ObstacleDistanceGrid&        dist_grid)
{
    // TODO #1: Implement processOdometry - check action_model.cpp
    bool moved = action_model_.processOdometry(odom);
    nav2_msgs::msg::ParticleCloud resampled_particles = systematicResample();
    nav2_msgs::msg::ParticleCloud prior = moved ? propagate(resampled_particles) : resampled_particles;
    particle_cloud_ = weightParticles(prior, scan, dist_grid);
    pose_estimate_ = computeBestEstimate(particle_cloud_);

    return pose_estimate_;
}

nav2_msgs::msg::ParticleCloud ParticleFilter::systematicResample() const
{
    // TODO #2: Resample the particles
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);


    return resampled;
}

// Propagate each particle through ActionModel
nav2_msgs::msg::ParticleCloud ParticleFilter::propagate(const nav2_msgs::msg::ParticleCloud& resampled_particles)
{
    nav2_msgs::msg::ParticleCloud prior;
    prior.particles.reserve(num_particles_);

    for (const auto& particle : resampled_particles.particles) {
        nav2_msgs::msg::Particle q = particle;
        // TODO #3: Propagate each resampled particles through the action model
        // Implement propagateParticle function in action_model.cpp
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
        // TODO #4: Weight the particles using the sensor model
        // Implement likelihood function in sensor_model.cpp
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

// Compute pose estimate
geometry_msgs::msg::Pose ParticleFilter::computeBestEstimate() const
{
    double x=0.0, y=0.0, cos_sum=0.0, sin_sum=0.0;

    // TODO #5: Compute the best estimate pose
    // Fill in x, y, cos_sum, sin_sum
    
    geometry_msgs::msg::Pose est;
    est.position.x = x;
    est.position.y = y;
    est.position.z = 0.0;
    setOrientationFromYaw(est, std::atan2(sin_sum, cos_sum));
    return est;
}