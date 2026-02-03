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
    // Delta translation in the odom frame
    const double dx = odom.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = odom.pose.pose.position.y - prev_odom_.pose.pose.position.y;

    // Extract planar headings
    const double theta_prev = yawFromQuaternion(prev_odom_.pose.pose.orientation);
    const double theta_curr = yawFromQuaternion(odom.pose.pose.orientation);


    // TODO #1: Compute the rotation-translation-rotation motion components
    //          Replace following 0s with the correct expressions
    double delta_trans = std::hypot(dx, dy);
    double delta_theta = wrapToPi(theta_curr - theta_prev);
    rot1_  = wrapToPi(std::atan2(dy, dx) - theta_prev);
    trans_ = delta_trans;
    rot2_  = wrapToPi(theta_curr - theta_prev - rot1_);

    const bool moved =
        (delta_trans >= min_trans_) ||
        (std::abs(delta_theta) >= min_rot_);

    if (moved) {
        // TODO #2: calcuate standard deviations (alpha-model)
        // Hint: use k1_ and k2_ member variables
        rot1_std_  = k1_ * std::abs(rot1_) + k2_ * trans_;
        trans_std_ = k1_ * trans_ + k2_ * (std::abs(rot1_) + std::abs(rot2_));
        rot2_std_  = k1_ * std::abs(rot2_) + k2_ * trans_;
    }

    prev_odom_ = odom;

    return moved;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    /// TODO #3: Sample noisy motion components
    // Hint: use std::normal_distribution
    // Hint: random_gen is available as a member variable
    // e.g., std::normal_distribution<double> dist(mean, std_dev);
    //       double sample = dist(random_gen);
    //       where mean should be the odometry
    // Replace 0s with the correct expressions
    std::normal_distribution<double> dist_rot1(rot1_, std::max(0.0, rot1_std_));
    std::normal_distribution<double> dist_trans(trans_, std::max(0.0, trans_std_));
    std::normal_distribution<double> dist_rot2(rot2_, std::max(0.0, rot2_std_));

    const double sampled_rot1  = dist_rot1(random_gen);
    const double sampled_trans = dist_trans(random_gen);
    const double sampled_rot2  = dist_rot2(random_gen);

    // TODO #4: Update particle position and orientation based on sampled motion
    // Replace 0s with the correct expressions
    geometry_msgs::msg::Pose new_pose = pose;
    const double theta = yawFromQuaternion(pose.orientation);
    const double theta_prime = wrapToPi(theta + sampled_rot1);
    new_pose.position.x = pose.position.x + sampled_trans * std::cos(theta_prime);
    new_pose.position.y = pose.position.y + sampled_trans * std::sin(theta_prime);
    const double theta_final = wrapToPi(theta_prime + sampled_rot2);
    setOrientationFromYaw(new_pose, theta_final);

    return new_pose;

}
