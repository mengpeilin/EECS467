#include "action_model.hpp"
#include "localization_utils.hpp"

#include <random>
#include <utility>
#include <cmath>
#include <algorithm>

using namespace mbot_localization;

ActionModel::ActionModel()
{
    std::random_device rd;
    random_gen = std::mt19937(rd());
}

void ActionModel::setOdomReference(const nav_msgs::msg::Odometry& odom)
{
    prev_odom_ = odom;
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
    const double delta_trans = std::hypot(dx, dy);
    const double delta_theta = wrapToPi(theta_curr - theta_prev);

    if (delta_trans < 1e-12) {
        // essentially pure rotation
        rot1_  = 0.0;
        trans_ = 0.0;
        rot2_  = delta_theta;
    } else {
        // direction of translation in world/odom frame
        const double direction = std::atan2(dy, dx);
        rot1_  = wrapToPi(direction - theta_prev);
        trans_ = delta_trans;
        rot2_  = wrapToPi(theta_curr - theta_prev - rot1_);
    }

    const bool moved =
        (delta_trans >= min_trans_) ||
        (std::abs(delta_theta) >= min_rot_);

    if (moved) {
        // TODO #2: calcuate standard deviations (alpha-model)
        // 用平方和再开方，稳很多，避免线性相加导致噪声过大
        const double rot1_sq  = rot1_ * rot1_;
        const double rot2_sq  = rot2_ * rot2_;
        const double trans_sq = trans_ * trans_;

        rot1_std_  = std::sqrt(std::max(0.0, k1_ * rot1_sq + k2_ * trans_sq));
        trans_std_ = std::sqrt(std::max(0.0, k1_ * trans_sq + k2_ * (rot1_sq + rot2_sq)));
        rot2_std_  = std::sqrt(std::max(0.0, k1_ * rot2_sq + k2_ * trans_sq));

        // 防止 std 为 0 导致采样退化成常数（可选但很安全）
        rot1_std_  = std::max(rot1_std_,  1e-12);
        trans_std_ = std::max(trans_std_, 1e-12);
        rot2_std_  = std::max(rot2_std_,  1e-12);
    }

    prev_odom_ = odom;
    return moved;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    /// TODO #3: Sample noisy motion components
    std::normal_distribution<double> rot1_dist(rot1_,  rot1_std_);
    std::normal_distribution<double> trans_dist(trans_, trans_std_);
    std::normal_distribution<double> rot2_dist(rot2_,  rot2_std_);

    const double sampled_rot1  = rot1_dist(random_gen);
    const double sampled_trans = trans_dist(random_gen);
    const double sampled_rot2  = rot2_dist(random_gen);

    // TODO #4: Update particle position and orientation based on sampled motion
    const double theta = yawFromQuaternion(pose.orientation);

    geometry_msgs::msg::Pose new_pose = pose;

    const double theta1 = theta + sampled_rot1;
    new_pose.position.x = pose.position.x + sampled_trans * std::cos(theta1);
    new_pose.position.y = pose.position.y + sampled_trans * std::sin(theta1);

    const double new_yaw = wrapToPi(theta + sampled_rot1 + sampled_rot2);
    setOrientationFromYaw(new_pose, new_yaw);

    return new_pose;
}
