#include "so_arm_driver/so_arm_driver.hpp"

namespace SOArm {

SoArmDriver::SoArmDriver() {
    // Consider servos from 1 to 6 to be the standard
    std::iota(m_servosIds.begin(), m_servosIds.end(), 1);
    setUpJointLimits();
}

SoArmDriver::~SoArmDriver()
{
}

void SoArmDriver::updateState() {
    for (size_t i = 0; i < JOINT_NUMBER; i++) {
        m_servos.FeedBack(m_servosIds[i]);
        m_state.encoder[i] = m_servos.ReadPos(-1);
        m_state.pos[i] = encoder2Pos(m_state.encoder[i], i);
        m_state.vel[i] = m_servos.ReadSpeed(-1);
        m_state.load[i] = m_servos.ReadLoad(-1);
        m_state.voltage[i] = m_servos.ReadVoltage(-1);
        m_state.temperature[i] = m_servos.ReadTemper(-1);
    }
}
void SoArmDriver::setUpJointLimits() {

    for (size_t i = 0; i < JOINT_NUMBER; i++) {
        // Read encoder's limit positions from EEPROM
        m_limits.encoder[i].min = m_servos.ReadMinAngleLimit(m_servosIds[i]);
        m_limits.encoder[i].max = m_servos.ReadMaxAngleLimit(m_servosIds[i]);

        // Get joint angle limits
        std::pair<double, double> angleLimits = getJointAngleLimit(i);
        m_limits.angle[i].min = angleLimits.first;
        m_limits.angle[i].max = angleLimits.second;

        // Set up the joint calibration considering a linear relation between angle and encoder
        // 12 bits = 4095 = 360° = 2 pi
        setJointPosition2EncoderConsts(i);
    }
}

std::pair<double, double> SoArmDriver::getJointAngleLimit(int joint) {
// TODO: Automaticly get joint limits from robot URDF
    std::pair<double, double> limits;
    switch (joint) {
    case 0:
        limits.first = -1.920;
        limits.second= 1.920;
        return limits;
    case 1:
        limits.first = -1.745;
        limits.second= 1.745;
        return limits;
    case 2:
        limits.first = -1.690;
        limits.second= 1.580;
        return limits;
    case 3:
        limits.first = -1.658;
        limits.second= 1.658;
        return limits;
    case 4:
        limits.first = -2.744;
        limits.second= 2.841;
        return limits;
    case 5:
        limits.first = -0.175;
        limits.second= 1.745;
        return limits;
    default:
        RCLCPP_ERROR_STREAM(m_logger, "Requested limits from un unknow joint");
        return limits;
    }
}

void SoArmDriver::setJointPosition2EncoderConsts(const int joint) {
    const double encMax = static_cast<double>(m_limits.encoder[joint].max);
    const double encMin = static_cast<double>(m_limits.encoder[joint].min);
    const double angleMax = m_limits.angle[joint].max;
    const double angleMin = m_limits.angle[joint].min;
    m_consts[joint].slope = (angleMax - angleMin) / (encMax - encMin);
    m_consts[joint].intercept = (encMax * angleMin - encMax * angleMax) / (encMax - encMin);
}



}  // namespace so_arm_driver
