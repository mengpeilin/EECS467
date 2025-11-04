#include "action_model.hpp"
#include "localization_utils.hpp"

#include <random>
#include <utility>
#include <cmath>

using namespace mbot_localization;

ActionModel::ActionModel()
{
    std::random_device rd;
    random_gen = std::mt19937(rd());
}

void ActionModel::setOdomReference(const nav_msgs::msg::Odometry& odom)
{
    prev_odom_   = odom;
}

bool ActionModel::processOdometry(const nav_msgs::msg::Odometry& odom)
{

    const double dx = odom.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = odom.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    const double theta_prev = yawFromQuaternion(prev_odom_.pose.pose.orientation);
    const double theta_curr = yawFromQuaternion(odom.pose.pose.orientation);

    // TODO #1: Compute the rotation-translation-rotation motion components
    // Replace 0s with the correct expressions
    double delta_trans = 0;
    double delta_theta = 0;
    rot1_ = 0;
    trans_ = 0;
    rot2_ = 0;

    const bool moved =
        (delta_trans >= min_trans_) ||
        (std::abs(delta_theta) >= min_rot_);

    if (moved) {
        // TODO #2: calcuate standard deviations (alpha-model)
        // Hint: use k1_ and k2_ member variables
        rot1_std_  = 0;
        trans_std_ = 0;
        rot2_std_  = 0;
    }

    prev_odom_ = odom;

    return moved;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    // TODO #3: Sample noisy motion components
    // Hint: use std::normal_distribution
    // Hint: random_gen is available as a member variable
    // e.g., std::normal_distribution<double> dist(mean, std_dev);
    //       double sample = dist(random_gen);
    //       where mean should be the odometry
    // Replace 0s with the correct expressions
    double sampled_rot1  = 0.0;
    double sampled_trans = 0.0;
    double sampled_rot2  = 0.0; 

    // TODO #4: Update particle position and orientation based on sampled motion
    // Replace 0s with the correct expressions
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = 0;
    new_pose.position.y = 0;
    setOrientationFromYaw(new_pose, wrapToPi(0));

    return new_pose;
}
