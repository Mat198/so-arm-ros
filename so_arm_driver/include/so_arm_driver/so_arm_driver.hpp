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

  void updateState();

  void setTarget(JointArray pos, JointArray vel);

  void calibrate();
  
private:

  void setUpJointLimits();

  std::pair<double,double> getJointAngleLimit(int joint);

  void setJointPosition2EncoderConsts(int joint);

  // Convert encoder position to angle in radians
  double encoder2Pos(const int encoderRead, int joint);

  // Convert angle in radians to encoder position 
  int pos2Encoder(double pos);

  JointArray vel2Encoder(JointArray vel);

  SCSCL m_servos;
  State m_state;
  Target m_target;
  JointIntArray m_servosIds;
  JointLimits m_limits;
  JointConsts m_consts;

};

}  // namespace SOArm

#endif  // SO_ARM_DRIVER_HPP
