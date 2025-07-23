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
        jointStateMsg.position.reserve(JOINT_NUMBER);
        jointStateMsg.velocity.reserve(JOINT_NUMBER);
        jointStateMsg.effort.reserve(JOINT_NUMBER);
        for (size_t i = 1; i <= JOINT_NUMBER; i++) {
            jointStateMsg.position.push_back(state.pos[JOINT_NUMBER - i]);
            jointStateMsg.velocity.push_back(state.vel[JOINT_NUMBER - i]);
            jointStateMsg.effort.push_back(state.load[JOINT_NUMBER - i]);
        }
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
