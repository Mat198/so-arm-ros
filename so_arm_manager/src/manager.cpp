#include "so_arm_manager/manager.hpp"

namespace SOArm {

Manager::Manager() : 
    Node("so_arm_manager"),
    m_logger(this->get_logger()),
    m_clock(this->get_clock()),
    m_torqueEnableCommand(true)
{
    RCLCPP_INFO_STREAM(m_logger, "SO-ARM Manager created! Waiting initialization...");
}

void Manager::init() {

    using namespace std::placeholders; // _1 and _2

    // Starting timers
    m_enableTorqueTimer = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&Manager::publishTorqueEnableCommand, this)
    );
    
    // Starting publisher 
    m_enableTorqueCommandPub = this->create_publisher<SetDynamicInterface>(
        "motor_torque_enable_controller/commands", 10
    );

    // Starting subscribers
    m_dynamicStateSub = this->create_subscription<DynamicJointStateMsg>(
        "dynamic_joint_states", 10, std::bind(&Manager::dynamicStateCallback, this, _1)
    );

    // Starting services
    m_userCbGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_enableTorqueSrv = this->create_service<TriggerSrv>(
        "so_arm/enable_torque", std::bind(&Manager::enableTorqueCallback, this, _1, _2), 
        rmw_qos_profile_services_default, m_userCbGroup
    );

    m_disableTorqueSrv = this->create_service<TriggerSrv>(
        "so_arm/disable_torque", std::bind(&Manager::disableTorqueCallback, this, _1, _2), 
        rmw_qos_profile_services_default, m_userCbGroup
    );

    // Starting clients
    m_controllerCbGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
    m_switchControllersClient = std::make_shared<SyncClient<SwitchControllerSrv>>(
        shared_from_this(), "controller_manager/switch_controller", m_controllerCbGroup
    );

    m_listControllersClient = std::make_shared<SyncClient<ListControllersSrv>>(
        shared_from_this(), "controller_manager/list_controllers", m_controllerCbGroup
    );

    RCLCPP_INFO_STREAM(m_logger, "SO-ARM Manager initialized!");
}

void Manager::publishTorqueEnableCommand() {


    SetDynamicInterface enableTorqueMsg;
    enableTorqueMsg.interface_groups = JOINT_NAMES;
    
    InterfaceValue value;
    value.interface_names = {"enable"};
    value.values = {static_cast<double>(m_torqueEnableCommand)};
    enableTorqueMsg.interface_values = std::vector<InterfaceValue>(JOINT_NUMBER, value);

    m_enableTorqueCommandPub->publish(enableTorqueMsg);
}

void Manager::enableTorqueCallback(
    const std::shared_ptr<TriggerSrv::Request> /* request */,
    std::shared_ptr<TriggerSrv::Response> response
) {

    const bool torqueEnabled = std::all_of(
        std::begin(m_armState.enabled), std::end(m_armState.enabled), [](bool i) {return i;});
    if (torqueEnabled) {
        response->message = "Servos already have torque enabled!";
        response->success = torqueEnabled;
    }

    bool success = activateControllers();
    if (success) {
        response->message = "Torque enabled for servos!";
        m_torqueEnableCommand = true;
        publishTorqueEnableCommand();
        RCLCPP_INFO_STREAM(m_logger, response->message);
    } else {
        response->message = "Failed to enable torque for servos.";
        RCLCPP_ERROR_STREAM(m_logger, response->message);
    }
    response->success = success;
}

void Manager::disableTorqueCallback(
    const std::shared_ptr<TriggerSrv::Request> /* request */,
    std::shared_ptr<TriggerSrv::Response> response
) {

    const bool torqueDisabled = std::all_of(
        std::begin(m_armState.enabled), std::end(m_armState.enabled), [](bool i) {return !i;});
    if (torqueDisabled) {
        response->message = "Servos already have torque disabled!";
        response->success = torqueDisabled;
    }

    bool success = deactivateControllers();
    if (success) {
        response->message = "Torque disabled for servos!";
        m_torqueEnableCommand = false;
        publishTorqueEnableCommand();
        RCLCPP_INFO_STREAM(m_logger, response->message);
    } else {
        response->message = "Failed to disabled torque for servos.";
        RCLCPP_ERROR_STREAM(m_logger, response->message);
    }
    response->success = success;
}
    
void Manager::dynamicStateCallback(const DynamicJointStateMsg::SharedPtr msg) {

    // Reading the state of each interface of each joint
    // TODO: Monitor the health of the joints

    for (const auto &joint : msg->interface_values) {
        for (size_t i = 0; i < JOINT_NUMBER; i++) {
            // Is there a way to not hardcode this?
           
            if (joint.interface_names[i] == "position") {
                m_armState.pos[i] = joint.values[i]; 
            }
            if (joint.interface_names[i] == "velocity") {
                m_armState.vel[i] = joint.values[i]; 
            }
            if (joint.interface_names[i] == "torque_enabled") {
                m_armState.enabled[i] = joint.values[i]; 
            }
            if (joint.interface_names[i] == "voltage") {
                m_armState.voltage[i] = joint.values[i]; 
            }
            if (joint.interface_names[i] == "load") {
                m_armState.load[i] = joint.values[i]; 
            }
            if (joint.interface_names[i] == "temperature") {
                m_armState.temperature[i] = joint.values[i]; 
            }
            if (joint.interface_names[i] == "move") {
                m_armState.move[i] = joint.values[i]; 
            }
        }
    }
}

bool Manager::activateControllers() {

    // Check if controllers are active
    bool armControllerActive = false;
    bool gripperControllerActive = false;
    bool status = checkControllerStatus(armControllerActive, gripperControllerActive);
    if (!status) {
        RCLCPP_ERROR_STREAM(m_logger, "Failed to check Arm and Gripper controllers.");
        return false;
    }

    SwitchControllerSrv::Request controllerRequest;
    SwitchControllerSrv::Response controllerResponse;
    if (!armControllerActive) {
        controllerRequest.activate_controllers.push_back("so_arm_controller");
    }
    if (!gripperControllerActive) {
        controllerRequest.activate_controllers.push_back("gripper_controller");
    }
    if (controllerRequest.activate_controllers.empty()) {
        RCLCPP_INFO_STREAM(m_logger, "Arm and Gripper controller already active!");
        return true;
    }
    controllerRequest.strictness = controllerRequest.STRICT;
    controllerRequest.timeout = rclcpp::Duration::from_seconds(1.0);
    controllerRequest.activate_asap = true;

    RCLCPP_INFO_STREAM(m_logger, "Enabling Arm and Gripper controllers...");
    const bool success = m_switchControllersClient->call(controllerRequest, controllerResponse);
    
    if (success) {
        RCLCPP_INFO_STREAM(m_logger, "Arm and Gripper controllers enabled!");
    } else {
        RCLCPP_ERROR_STREAM(m_logger, "Failed to enable Arm and Gripper controllers.");
    }
    return success;
}

bool Manager::deactivateControllers() {

    // Check if controllers are active
    bool armControllerActive = true;
    bool gripperControllerActive = true;
    bool status = checkControllerStatus(armControllerActive, gripperControllerActive);
    if (!status) {
        RCLCPP_ERROR_STREAM(m_logger, "Failed to check Arm and Gripper controllers.");
        return false;
    }
    
    SwitchControllerSrv::Request controllerRequest;
    SwitchControllerSrv::Response controllerResponse;
    if (armControllerActive) {
        controllerRequest.deactivate_controllers.push_back("so_arm_controller");
    }
    if (gripperControllerActive) {
        controllerRequest.deactivate_controllers.push_back("gripper_controller");
    }
    if (controllerRequest.deactivate_controllers.empty()) {
        RCLCPP_INFO_STREAM(m_logger, "Arm and Gripper controller already inactive!");
        return true;
    }
    controllerRequest.strictness = controllerRequest.STRICT;
    controllerRequest.timeout = rclcpp::Duration::from_seconds(1.0);
    controllerRequest.activate_asap = true;

    RCLCPP_INFO_STREAM(m_logger, "Disabling Arm and Gripper controllers...");
    const bool success = m_switchControllersClient->call(controllerRequest, controllerResponse);
    
    if (success) {
        RCLCPP_INFO_STREAM(m_logger, "Arm and Gripper controllers disabled!");
    } else {
        RCLCPP_ERROR_STREAM(m_logger, "Failed to disable Arm and Gripper controllers.");
    }
    return success;
}

bool Manager::checkControllerStatus(bool & armController, bool & gripperController) {
    
    ListControllersSrv::Request request; // Empty
    ListControllersSrv::Response response;
    bool success = m_listControllersClient->call(request, response);

    if (!success) {
        RCLCPP_ERROR_STREAM(m_logger, "Failed to check controllers.");
        return false;
    }

    for (const auto & controller : response.controller) {
        if (controller.name == "so_arm_controller") {
            armController = controller.state == "active";
            RCLCPP_INFO_STREAM(m_logger, "The 'so_arm_controller' is " << controller.state);
        }
        if (controller.name == "gripper_controller") {
            gripperController = controller.state == "active";
            RCLCPP_INFO_STREAM(m_logger, "The 'gripper_controller' is " << controller.state);
        }
    }
    return true;
}

} // namespace SOArm
