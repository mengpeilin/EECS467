#include "particle_filter.hpp"
#include "localization_utils.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>

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

// Main update cycle
geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&     odom,
                                                const sensor_msgs::msg::LaserScan& scan,
                                                const ObstacleDistanceGrid&        dist_grid)
{
    bool moved = action_model_.processOdometry(odom);
    nav2_msgs::msg::ParticleCloud resampled_particles = systematicResample();
    nav2_msgs::msg::ParticleCloud prior = moved ? propagate(resampled_particles) : resampled_particles;
    particle_cloud_ = weightParticles(prior, scan, dist_grid);
    pose_estimate_ = computeBestEstimate(particle_cloud_);

    return pose_estimate_;
}

// Systematic resampling: keep high-weight particles, discard low-weight ones
nav2_msgs::msg::ParticleCloud ParticleFilter::systematicResample() const
{

    // TODO #2: Resample the particles
    // Step 1: Build cumulative distribution of weights
    


    // Step 2: Handle edge case (if all particles have zero weight?)
    



    // Step 3: Normalize to probability [0, 1]
    


    // Step 4: Draw N equally-spaced samples and copy particles. 
    //You may find  std::uniform_real_distribution<double> unif() helpful.
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);
    //(fill in code below)





   
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

// Compute pose estimate (weighted mean on SE2)
geometry_msgs::msg::Pose ParticleFilter::computeBestEstimate(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    double x=0.0, y=0.0, cos_sum=0.0, sin_sum=0.0;

    // TODO #5: Compute the best estimate pose
    // Loop through all p in cloud.particles. Fill in x, y, cos_sum, sin_sum







    geometry_msgs::msg::Pose est;
    est.position.x = x;
    est.position.y = y;
    est.position.z = 0.0;
    setOrientationFromYaw(est, std::atan2(sin_sum, cos_sum));
    return est;
}
