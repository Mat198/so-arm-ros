#ifndef SO_ARM_CONSTS_HPP
#define SO_ARM_CONSTS_HPP

#include "rclcpp/rclcpp.hpp"

namespace SOArm {

static constexpr size_t JOINT_NUMBER = 6;
static const size_t STEPS_PER_REVOLUTION = 4096;
static const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"      
};

using JointVector = std::vector<double>;
using JointArray = std::array<double, JOINT_NUMBER>;
using JointIntArray = std::array<int, JOINT_NUMBER>;
using JointBoolArray = std::array<bool, JOINT_NUMBER>;

struct Command {
    JointArray pos;
    JointArray vel;
    JointArray acc;
    JointArray enable;
};

struct State {
    JointBoolArray enabled;
    JointArray pos;
    JointArray vel;
    JointArray acc;
    JointIntArray steps;
    JointIntArray stepsVel;
    JointArray load;
    JointArray voltage;
    JointArray temperature;
    JointArray move;
};

template<typename T>
struct Limit {
    T min;
    T max;
};

struct JointLimits {
    std::array<Limit<double>, JOINT_NUMBER> angle;
    std::array<Limit<int>, JOINT_NUMBER> encoder;
};


// Convert consts in the format y = a*x + b
struct ConvertConst {
    double slope; //  constant
    double intercept; // intercep
};
using JointConsts = std::array<ConvertConst, JOINT_NUMBER>;

}  // namespace SOArm
#endif  // SO_ARM_CONSTS_HPP
