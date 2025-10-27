#include "so_arm_hardware_interface/arm_interface.hpp"

namespace SOArm {

hardware_interface::CallbackReturn SOArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info
) {
    RCLCPP_INFO(m_logger, "Starting SO-ARM hardware interface...");

    // Setting vectors to Joint size
    m_state.positions = std::vector<double>(JOINT_NUMBER, 0.0);
    m_state.velocities = m_state.effort = m_state.positions;
    m_target = m_state;

    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS
    ) {
        RCLCPP_FATAL(m_logger, "Failed to init hardware interface.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify if the interfaces are correct
    for (const hardware_interface::ComponentInfo &joint : info.joints) {
        // Checa pelo número de interfaces de comando
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                m_logger,
                "Joint '%s' has %zu command interfaces found."
                " 3 expected (position, velocity and acceleration).",
                joint.name.c_str(),
                joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checa pela interface de comando de posição
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                m_logger,
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checa pela interface de velocidade das juntas
        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                m_logger,
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(),
                joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checa pelo número de interfaces de sensores de velocidade e posição das juntas
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                m_logger,
                "Joint '%s' has %zu state interface."
                " 3 expected (position, velocity and acceleration).",
                joint.name.c_str(), 
                joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                m_logger,
                "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                m_logger,
                "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

    }

    RCLCPP_INFO_STREAM(m_logger, "Successfully started!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SOArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
    RCLCPP_INFO_STREAM(m_logger, "Configuring... please wait...");
    // TODO: Calibrate driver here
    // m_driver.calibrate()
    RCLCPP_INFO_STREAM(m_logger, "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SOArmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &m_state.positions[i]
            )
        );

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &m_state.velocities[i]
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SOArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); ++i) {

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &m_target.positions[i]
            )
        );
        RCLCPP_INFO_STREAM(
            m_logger, "Position command interface configured for joint " << i << "!"
        );

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &m_target.velocities[i]
            )
        );
        RCLCPP_INFO_STREAM(
            m_logger, "Velocity command interface configured for joint " << i << "!"
        );

    }

    return command_interfaces;
}

hardware_interface::CallbackReturn SOArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
    RCLCPP_INFO(m_logger, "Activating... please wait...");
    // TODO: Activate?
    RCLCPP_INFO(m_logger, "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SOArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
    RCLCPP_INFO(m_logger, "Deactivating... please wait...");
    // Call destructor direct to ensure the comunication is closed
    // m_driver.~SoArmDriver();
    RCLCPP_INFO(m_logger, "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SOArmHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
    RCLCPP_INFO(m_logger, "Cleanning up... please wait...");
    RCLCPP_INFO(m_logger, "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SOArmHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
    RCLCPP_INFO(m_logger, "Shuting down... please wait...");
    RCLCPP_INFO(m_logger, "Successfully shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SOArmHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/
) {

    // Here we can't just do m_state.position = state.pos (Weird values appear!)
    // because we need to keep the same memory adress asigned before to the state interface
    // We need to update each value individually
    const auto state = m_driver.updateState();
    for (uint i = 0; i < info_.joints.size(); ++i) {
        m_state.positions[i] = state.pos[i];
        m_state.velocities[i] = state.vel[i];
        m_state.effort[i] = state.load[i];
    }
    
    // Set the first target to the current position and not to zero. 
    // Prevents the robot to move unexpectly.
    if (!m_targetInitialized) {
        for (uint i = 0; i < info_.joints.size(); ++i) {
            m_target.positions[i] = m_state.positions[i];
            m_target.velocities[i] = m_state.velocities[i];
            m_target.effort[i] = m_state.effort[i];
        }
        RCLCPP_WARN_STREAM(m_logger, std::setprecision(3) << std::fixed  
            << "Initial state is {" << m_state.positions[0] << ", " << m_state.positions[1] << ", "
            << m_state.positions[2] << ", " << m_state.positions[3] << ", " 
            << m_state.positions[4] << "}"
        );
        m_targetInitialized = true;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SOArmHardwareInterface::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/
) {
    if (!m_targetInitialized) {
        // We don't send any command until the target is actually started.
        // Prevent the robot moving immediately to zero when it's started
        return hardware_interface::return_type::OK;
    }
    
    const std::vector<double> pos = m_target.positions; 
    const std::vector<double> vel = m_target.velocities; 
    m_driver.setTarget(pos, vel);
    return hardware_interface::return_type::OK;
}

}  // namespace SOArm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(SOArm::SOArmHardwareInterface, hardware_interface::SystemInterface)
