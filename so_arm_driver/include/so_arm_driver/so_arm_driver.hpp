#ifndef SO_ARM_DRIVER_HPP
#define SO_ARM_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include "so_arm_driver/so_arm_data.hpp"
#include "sc_servo/SCServo.hpp"

namespace SOArm {

class SoArmDriver {

public:
  SoArmDriver();

  ~SoArmDriver();

  State updateState();

  void setTarget(const JointArray &pos, const JointArray &vel);

  void enableMotorTorque(const JointArray &enable);

  // TODO: Function to check if target is inside limits

  // TODO: Implement
  void calibrate();
  
private:

  void setUpJointLimits();

  std::pair<double,double> getJointAngleLimit(int joint);

  void setJointPosition2EncoderConsts(int joint);

  // Convert encoder position to angle in radians
  double encoder2Pos(const int encoder, const int joint);

  // Convert angle in radians to encoder position 
  uint16_t pos2Encoder(const double pos, const int joint);

  // Convert velocity from steps/s to rad/s
  double steps2Vel(const int steps);

  // Convert velocity from rad/s to steps/s
  uint16_t vel2steps(const double vel);

  SCSCL m_servos;
  State m_state;
  JointIntArray m_servosIds;
  JointLimits m_limits;

  // Stores the conversion constants from encoder steps to position in degrees. 
  JointConsts m_consts;

  const rclcpp::Logger m_logger = rclcpp::get_logger("so_arm_driver");
  rclcpp::Clock m_sysClock = rclcpp::Clock(RCL_SYSTEM_TIME);

  const int ERROR_INTERVAL_MS = 5000;
};

}  // namespace SOArm

#endif  // SO_ARM_DRIVER_HPP
