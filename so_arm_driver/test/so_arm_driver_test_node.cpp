#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "so_arm_driver/so_arm_driver.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace SOArm {

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointState = sensor_msgs::msg::JointState;
using TriggerSrv = std_srvs::srv::Trigger;
using SetJointSrv = example_interfaces::srv::AddTwoInts;
using JointEncodersMsg = std_msgs::msg::Int16MultiArray;

using std::placeholders::_1;
using std::placeholders::_2;

class SoArmDemo : public rclcpp::Node {
public:
    SoArmDemo() : Node("so_arm_demo_node") {
        m_trajectorySub = this->create_subscription<JointTrajectoryPoint>(
            "/so_arm/target", 10, std::bind(&SoArmDemo::trajectoryCallback, this, _1));
        m_publishTimer = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&SoArmDemo::publishCallback, this)
        );
        m_jointStatesPub = this->create_publisher<JointState>("/joint_states", 10);
        m_jointEncoderPub = this->create_publisher<JointEncodersMsg>("/joint_encoders", 10);
        
        m_enableCalibrationService = this->create_service<TriggerSrv>(
            "/so_arm/enable_calibration", 
            std::bind(&SoArmDemo::enableCalibrationCallback, this, _1, _2)
        );

        m_disableCalibrationService = this->create_service<TriggerSrv>(
            "/so_arm/disable_calibration", 
            std::bind(&SoArmDemo::disableCalibrationCallback, this, _1, _2)
        );

        m_setJointMinService = this->create_service<SetJointSrv>(
            "/so_arm/set_min_angle", std::bind(&SoArmDemo::setMinJointAngleCallback, this, _1, _2)
        );
        m_setJointMaxService = this->create_service<SetJointSrv>(
            "/so_arm/set_max_angle", std::bind(&SoArmDemo::setMaxJointAngleCallback, this, _1, _2)
        );
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

        JointEncodersMsg jointEncodersMsg;
        jointEncodersMsg.data = std::vector<int16_t>(state.steps.begin(), state.steps.end());
        m_jointEncoderPub->publish(jointEncodersMsg);
    }

    void enableCalibrationCallback(
        const std::shared_ptr<TriggerSrv::Request> /* request */,
        std::shared_ptr<TriggerSrv::Response> response
    ) {
        const JointArray set = {1,1,1,1,1,1};
        m_driver.setCalibrationMode(set);
        response->message = "Calibration mode enabled!";
        response->success = true;
        RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    }

    void disableCalibrationCallback(
        const std::shared_ptr<TriggerSrv::Request> /* request */,
        std::shared_ptr<TriggerSrv::Response> response
    ) {
        const JointArray set = {0,0,0,0,0,0};
        m_driver.setCalibrationMode(set);
        response->message = "Calibration mode disabled!";
        response->success = true;
        RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    }

    void setMinJointAngleCallback(
        const std::shared_ptr<SetJointSrv::Request> request,
        std::shared_ptr<SetJointSrv::Response> response
    ) {
        const int joint = request->a; // Joint
        if (joint >= 6 || joint < 0) {
            response->sum = -1;
            return;
        }
        m_driver.setJointMin(joint);
        response->sum = 0;
        RCLCPP_INFO_STREAM(this->get_logger(), "Min angle for joint " << joint << " updated!");
    }
    
    void setMaxJointAngleCallback(
        const std::shared_ptr<SetJointSrv::Request> request,
        std::shared_ptr<SetJointSrv::Response> response
    ) {
        const int joint = request->a; // Joint
        if (joint >= 6 || joint < 0) {
            response->sum = -1;
            return;
        }
        m_driver.setJointMax(joint);
        response->sum = 0;
        RCLCPP_INFO_STREAM(this->get_logger(), "Max angle for joint " << joint << " updated!");
    }

private:
    SoArmDriver m_driver;
    rclcpp::TimerBase::SharedPtr m_publishTimer;

    rclcpp::Publisher<JointState>::SharedPtr m_jointStatesPub;
    rclcpp::Publisher<JointEncodersMsg>::SharedPtr m_jointEncoderPub;

    rclcpp::Subscription<JointTrajectoryPoint>::SharedPtr m_trajectorySub;

    rclcpp::Service<TriggerSrv>::SharedPtr m_enableCalibrationService;
    rclcpp::Service<TriggerSrv>::SharedPtr m_disableCalibrationService;
    rclcpp::Service<SetJointSrv>::SharedPtr m_setJointMinService;
    rclcpp::Service<SetJointSrv>::SharedPtr m_setJointMaxService;
};
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SOArm::SoArmDemo>());
    rclcpp::shutdown();
    return 0;
}
