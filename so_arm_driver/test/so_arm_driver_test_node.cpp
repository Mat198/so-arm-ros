#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "so_arm_driver/so_arm_driver.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace SOArm {

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointState = sensor_msgs::msg::JointState;
using std::placeholders::_1;

class SoArmDemo : public rclcpp::Node {
public:
    SoArmDemo() : Node("so_arm_demo_node") {
        m_trajectorySub = this->create_subscription<JointTrajectoryPoint>(
            "/so_arm/target", 10, std::bind(&SoArmDemo::trajectoryCallback, this, _1));
        m_publishTimer = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&SoArmDemo::publishCallback, this)
        );
        m_jointStatesPub = this->create_publisher<JointState>("/joint_states", 10);
    }

    void trajectoryCallback(const JointTrajectoryPoint::SharedPtr msg) {
        JointArray targetPos;
        std::copy_n(msg->positions.begin(), JOINT_NUMBER, targetPos.begin());
        JointArray targetVel;
        std::copy_n(msg->velocities.begin(), JOINT_NUMBER, targetVel.begin());
        m_driver.setTarget(targetPos, targetVel);
    }

    void publishCallback() {
        State state = m_driver.updateState();
        JointState jointStateMsg;
        jointStateMsg.name = JOINT_NAMES;
        jointStateMsg.header.frame_id = "so_arm";
        jointStateMsg.header.stamp = get_clock()->now();
        jointStateMsg.position = std::vector<double>(state.pos.begin(), state.pos.end());
        jointStateMsg.velocity = std::vector<double>(state.vel.begin(), state.vel.end());
        jointStateMsg.effort = std::vector<double>(state.load.begin(), state.load.end());
        m_jointStatesPub->publish(jointStateMsg);
    }

private:
    SoArmDriver m_driver;
    rclcpp::TimerBase::SharedPtr m_publishTimer;
    rclcpp::Publisher<JointState>::SharedPtr m_jointStatesPub;
    rclcpp::Subscription<JointTrajectoryPoint>::SharedPtr m_trajectorySub;
};
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SOArm::SoArmDemo>());
    rclcpp::shutdown();
    return 0;
}
