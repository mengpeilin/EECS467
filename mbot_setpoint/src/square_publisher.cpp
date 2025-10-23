#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_setpoint/msg/pose2_d_array.hpp"

class SquarePublisher : public rclcpp::Node {
public:
    SquarePublisher() : Node("square_publisher") {
        rclcpp::QoS qos = rclcpp::QoS(10).transient_local();
        pub_ = this->create_publisher<mbot_setpoint::msg::Pose2DArray>("/goal_pose_array", qos);

        mbot_setpoint::msg::Pose2DArray goal_array;
        goal_array.poses = {
            pose(1.0, 0.0, 0.0),
            pose(1.0, 1.0, M_PI_2),
            pose(0.0, 1.0, M_PI),
            pose(0.0, 0.0, 0.0)
        };

        pub_->publish(goal_array);
        RCLCPP_INFO(this->get_logger(), "Published square trajectory with %zu poses.", goal_array.poses.size());
    }

private:
    rclcpp::Publisher<mbot_setpoint::msg::Pose2DArray>::SharedPtr pub_;

    static geometry_msgs::msg::Pose2D pose(double x, double y, double theta) {
        geometry_msgs::msg::Pose2D p;
        p.x = x;
        p.y = y;
        p.theta = theta;
        return p;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePublisher>());
    rclcpp::shutdown();
    return 0;
}
