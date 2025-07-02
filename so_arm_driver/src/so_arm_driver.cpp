#include "so_arm_driver/so_arm_driver.hpp"

namespace SOArm {

SoArmDriver::SoArmDriver() {
    // Consider servos from 1 to 6 to be the standard
    std::iota(m_servosIds.begin(), m_servosIds.end(), 1);
    readJointLimits();
    calibrate();
}

SoArmDriver::~SoArmDriver()
{
}

void SoArmDriver::readJointLimits() {

    for (size_t i = 0; i < JOINT_NUMBER; i++) {
        m_limits.encoder[i].min = m_servos.ReadMinAngleLimit(m_servosIds[i]);
        m_limits.encoder[i].max = m_servos.ReadMaxAngleLimit(m_servosIds[i]);
        // std::pair<double, double> angleLimits = getJointAngleLimit(i);
        // m_limits.angle[i].min = angleLimits.first;
        // m_limits.angle[i].max = angleLimits.second;
    }
}

// std::pair<double, double> getJointAngleLimit(int joint) {
// // TODO: Implementar para pegar a ângulo automaticamente do parâmetro da moveit
// }

double SoArmDriver::encoder2Pos(const int encoder, const int joint) {
    return m_consts[joint].slope * encoder + m_consts[joint].intercept;
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


}  // namespace so_arm_driver
