#ifndef SO_ARM_MANAGER_HPP
#define SO_ARM_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "so_arm_manager/sync_client_caller.hpp"
#include "so_arm_driver/so_arm_data.hpp"

namespace SOArm {

using TriggerSrv = std_srvs::srv::Trigger;
using SetDynamicInterface = control_msgs::msg::DynamicInterfaceGroupValues;
using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;
using DynamicJointStateMsg = control_msgs::msg::DynamicJointState;
using InterfaceValue = control_msgs::msg::InterfaceValue;
using ListControllersSrv = controller_manager_msgs::srv::ListControllers;

class Manager : public rclcpp::Node {

public:
    Manager();

    void init();

private:

    void publishTorqueEnableCommand();

    void enableTorqueCallback(
        const std::shared_ptr<TriggerSrv::Request> request,
        std::shared_ptr<TriggerSrv::Response> response
    );

    void disableTorqueCallback(
        const std::shared_ptr<TriggerSrv::Request> request,
        std::shared_ptr<TriggerSrv::Response> response
    );

    void dynamicStateCallback(const DynamicJointStateMsg::SharedPtr msg);

    bool checkControllerStatus(bool & armController, bool & gripperController);

    bool activateControllers();

    bool deactivateControllers();

    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;

    rclcpp::CallbackGroup::SharedPtr m_controllerCbGroup;
    rclcpp::CallbackGroup::SharedPtr m_userCbGroup;

    rclcpp::Publisher<SetDynamicInterface>::SharedPtr m_enableTorqueCommandPub;

    rclcpp::Subscription<DynamicJointStateMsg>::SharedPtr m_dynamicStateSub;

    rclcpp::Service<TriggerSrv>::SharedPtr m_disableTorqueSrv;
    rclcpp::Service<TriggerSrv>::SharedPtr m_enableTorqueSrv;

    rclcpp::TimerBase::SharedPtr m_enableTorqueTimer;

    SyncClient<SwitchControllerSrv>::SharedPtr m_switchControllersClient;
    SyncClient<ListControllersSrv>::SharedPtr m_listControllersClient;

    State m_armState;
    std::atomic_bool m_torqueEnableCommand;

};
}  // namespace SOArm
#endif  // SO_ARM_MANAGER_HPP
