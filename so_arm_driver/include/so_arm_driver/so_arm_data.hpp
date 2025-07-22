#ifndef SO_ARM_CONSTS_HPP
#define SO_ARM_CONSTS_HPP

#include "rclcpp/rclcpp.hpp"

namespace SOArm {

static constexpr size_t JOINT_NUMBER = 6;
static const size_t STEPS_PER_REVOLUTION = 4096;
using JointArray = std::array<double, JOINT_NUMBER>;
using JointIntArray = std::array<double, JOINT_NUMBER>;

struct Target {
    JointArray pos;
    JointArray vel;
    JointArray acc;
};

struct State {
    JointArray pos;
    JointArray vel;
    JointArray acc;
    JointIntArray encoder;
    JointIntArray load;
    JointIntArray voltage;
    JointIntArray temperature;
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
