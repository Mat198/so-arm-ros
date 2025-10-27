#include "so_arm_driver/so_arm_driver.hpp"

namespace SOArm {

SoArmDriver::SoArmDriver() {
    RCLCPP_INFO_STREAM(m_logger, "Starting SO-ARM Driver...");
    // Consider servos from 1 to 6 to be the standard
    std::iota(m_servosIds.begin(), m_servosIds.end(), 1);
    RCLCPP_INFO_STREAM(m_logger, "Ids set!");

    m_servos = SCSCL(0);
    std::string port = "/dev/ttyACM0";
    uint rate = 1000000;
    while(!m_servos.begin(rate, port.c_str()) && rclcpp::ok()) {
        RCLCPP_INFO_STREAM(
            m_logger, "Failed to connect to arm on port " << port << ". Retrying..."
        );
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO_STREAM(m_logger, "Connected with SO-ARM on port " << port << "at " << rate << "!");

    setUpJointLimits();
    RCLCPP_INFO_STREAM(m_logger, "SO-ARM Driver started!");
}

SoArmDriver::~SoArmDriver()
{
}

State SoArmDriver::updateState() {
    for (size_t i = 0; i < JOINT_NUMBER; i++) {
        const auto id = m_servosIds[i];
        m_servos.FeedBack(m_servosIds[i]);
        m_state.steps[i] = m_servos.ReadPos(id);
        m_state.pos[i] = encoder2Pos(m_state.steps[i], i);
        m_state.stepsVel[i] = m_servos.ReadSpeed(id);
        m_state.vel[i] = steps2Vel(m_state.stepsVel[i]);
        m_state.load[i] = m_servos.ReadLoad(id);
        m_state.voltage[i] = m_servos.ReadVoltage(id);
        m_state.temperature[i] = m_servos.ReadTemper(id);
    }
    return m_state;
}

void SoArmDriver::setTarget(JointVector pos, JointVector vel) {
    uint8_t ids[JOINT_NUMBER];
    uint16_t encPos[JOINT_NUMBER];
    uint16_t time[JOINT_NUMBER];
    uint16_t encVel[JOINT_NUMBER];

    if (pos.size() != JOINT_NUMBER || vel.size() != JOINT_NUMBER) {
        RCLCPP_ERROR_STREAM_THROTTLE(m_logger, m_sysClock, ERROR_INTERVAL_MS,
            "Received wrong number of joints in target! Received " << pos.size() 
            << " positions  and " << vel.size() << " expecting " << JOINT_NUMBER 
        );
        return;
    }

    for (size_t i = 0; i < JOINT_NUMBER; i++) {
        ids[i] = m_servosIds[i];
        encPos[i] = pos2Encoder(pos[i], i);
        time[i] = 0;
        encVel[i] = vel2steps(vel[i]);
    }
    
    m_servos.SyncWritePos(ids, JOINT_NUMBER,encPos, time, encVel);
}

void SoArmDriver::setTarget(JointArray pos, JointArray vel) {
    uint8_t ids[JOINT_NUMBER];
    uint16_t encPos[JOINT_NUMBER];
    uint16_t time[JOINT_NUMBER];
    uint16_t encVel[JOINT_NUMBER];

    for (size_t i = 0; i < JOINT_NUMBER; i++) {
        ids[i] = m_servosIds[i];
        encPos[i] = pos2Encoder(pos[i], i);
        time[i] = 0;
        encVel[i] = vel2steps(vel[i]);
    }
    
    m_servos.SyncWritePos(ids, JOINT_NUMBER,encPos, time, encVel);
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
        RCLCPP_INFO_STREAM(m_logger, "Joint " << i << " with ID " << m_servosIds[i] << " set!");
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
    m_consts[joint].intercept = (encMax * angleMin - encMin * angleMax) / (encMax - encMin);
}

double SoArmDriver::encoder2Pos(const int encoder, const int joint) {
    return m_consts[joint].slope * encoder + m_consts[joint].intercept;
}

uint16_t SoArmDriver::pos2Encoder(const double pos, const int joint) {
    return static_cast<uint16_t>((pos - m_consts[joint].intercept) / m_consts[joint].slope);
}

uint16_t SoArmDriver::vel2steps(const double vel) {
    // Converting rad/s to steps/s
    return static_cast<uint16_t>(abs(vel * STEPS_PER_REVOLUTION / (2 * M_PI))); 
}

double SoArmDriver::steps2Vel(const int steps) {
    return (2 * M_PI * static_cast<double>(steps)) / static_cast<double>(STEPS_PER_REVOLUTION);
}


}  // namespace so_arm_driver
