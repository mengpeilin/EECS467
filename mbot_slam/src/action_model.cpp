#include "action_model.hpp"
#include "slam_utils.hpp"

#include <random>
#include <utility>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace mbot_slam;

ActionModel::ActionModel()
{
    std::random_device rd;
    random_gen = std::mt19937(rd());
}

void ActionModel::setOdomReference(const nav_msgs::msg::Odometry& odom)
{
    prev_odom_   = odom;
    RCLCPP_INFO(rclcpp::get_logger("action_model"), "Initialized odometry reference.");
}

bool ActionModel::processOdometry(const nav_msgs::msg::Odometry& odom)
{
    // Delta translation in the odom frame
    const double dx = odom.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = odom.pose.pose.position.y - prev_odom_.pose.pose.position.y;

    // Extract planar headings
    const double theta_prev = yawFromQuaternion(prev_odom_.pose.pose.orientation);
    const double theta_curr = yawFromQuaternion(odom.pose.pose.orientation);

    const double delta_trans  = std::sqrt(dx*dx + dy*dy);
    const double delta_theta  = angleDiff(theta_curr, theta_prev);

    rot1_ = angleDiff(std::atan2(dy, dx), theta_prev);
    trans_ = delta_trans;
    rot2_ = angleDiff(delta_theta, rot1_);

    const bool moved = (delta_trans != 0.0) || (dx != 0.0) || (dy != 0.0);

    if (moved) {
        // Noise parameters (alpha-model)
        rot1_std_  = std::sqrt(k1_ * std::max(std::abs(rot1_), min_rot_));
        trans_std_ = std::sqrt(k2_ * std::max(std::abs(trans_), min_trans_));
        rot2_std_  = std::sqrt(k1_ * std::max(std::abs(rot2_), min_rot_));
    }

    prev_odom_ = odom;

    return moved;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    // Sample noisy motion components
    std::normal_distribution<double> n_rot1 (rot1_,  rot1_std_);
    std::normal_distribution<double> n_trans(trans_, trans_std_);
    std::normal_distribution<double> n_rot2 (rot2_,  rot2_std_);

    const double sr1   = n_rot1(random_gen);
    const double st    = n_trans(random_gen);
    const double sr2   = n_rot2(random_gen);

    // Current heading of the particle
    const double theta = yawFromQuaternion(pose.orientation);

    // Propagate
    geometry_msgs::msg::Pose new_pose = pose;
    new_pose.position.x += st * std::cos(theta + sr1);
    new_pose.position.y += st * std::sin(theta + sr1);
    setOrientationFromYaw(new_pose, wrapToPi(theta + sr1 + sr2));

    return new_pose;
}
