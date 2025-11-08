#ifndef SO_ARM_HARDWARE_INTERFACE_HPP
#define SO_ARM_HARDWARE_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "so_arm_driver/so_arm_driver.hpp"

namespace SOArm {

class SOArmHardwareInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SOArmHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info
    ) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period
    ) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period
    ) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:

    SoArmDriver m_driver;

    State m_state;
    Command m_command;

    const rclcpp::Logger m_logger = rclcpp::get_logger("so_arm_hardware_interface");
};

}  // namespace SOArm

#endif  // SO_ARM_HARDWARE_INTERFACE_HPP
