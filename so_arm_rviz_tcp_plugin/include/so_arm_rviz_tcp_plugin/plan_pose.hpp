#ifndef PLAN_POSE_HPP
#define PLAN_POSE_HPP

#include "so_arm_driver/so_arm_data.hpp"

#include <string>
#include <map>


namespace SOArm {

struct PlanPose {
    std::string name;
    // Saves joints position in radians
    JointArray jointPose;
};

using PathPlan = std::unordered_map<std::string, std::vector<double>>;

}  // namespace soArm
#endif  // PLAN_POSE_HPP
