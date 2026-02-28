#ifndef TCP_RVIZ_PLUGIN_HPP
#define TCP_RVIZ_PLUGIN_HPP

// ROS 2 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// RVIZ Plugin base
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/display_context.hpp>

// ROS 2 Interfaces
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>

// Qt Components
#include <QApplication>
#include <QClipboard>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QTreeWidget>

// Package includes
#include "so_arm_rviz_tcp_plugin/pose_tree_widget.hpp"
#include "so_arm_rviz_tcp_plugin/pose_delegate.hpp"
#include "so_arm_rviz_tcp_plugin/print_tools.hpp"
#include "so_arm_manager/sync_client_caller.hpp"

namespace SOArm {

using TriggerSrv = std_srvs::srv::Trigger;
using JointStateMsg = sensor_msgs::msg::JointState;

using SavedPoses = std::unordered_map<std::string, JointStateMsg>;
using Path = std::vector<std::string>;

using PoseMsg = geometry_msgs::msg::Pose;

using MotionSequenceRequest = moveit_msgs::msg::MotionSequenceRequest;
using MotionSequenceItem = moveit_msgs::msg::MotionSequenceItem;
using MotionPlanRequest = moveit_msgs::msg::MotionPlanRequest;
using MoveGroupSequenceAction = moveit_msgs::action::MoveGroupSequence;
using SequenceClientGoalHandle = rclcpp_action::ClientGoalHandle<MoveGroupSequenceAction>;

using Constraint = moveit_msgs::msg::Constraints;
using JointConstraint = moveit_msgs::msg::JointConstraint;

static const std::string PLANNING_GROUP = "so_arm";

class RobotTcp : public rviz_common::Panel {
    Q_OBJECT 

public:
    RobotTcp(QWidget *parent = 0);
    ~RobotTcp() override;

protected:
    
    void onInitialize() override;

    void save(rviz_common::Config config) const override;

    void load(const rviz_common::Config &config) override;

private Q_SLOTS:

    void addPose();

    void toggleTorque();

    void toggleRobot();

    void onPoseMoved();

    void onPoseChanged(QTreeWidgetItem *poseItem, int column);

Q_SIGNALS:

    void updateButtonText(const QString& message);

private:

    void setupGui();

    MotionSequenceRequest fillMotionSequenceRequest(const Path &path,const SavedPoses &targets);

    void sendRobotPath();

    void createPoseItem(const std::string &poseDefaultName, const std::vector<double> &pose);

    void updatePoseName(QTreeWidgetItem *poseItem);

    void updatePoseValues(QTreeWidgetItem *poseString);

    void jointStatesCallback(const JointStateMsg::SharedPtr msg);

    void disableTorqueCallback(rclcpp::Client<TriggerSrv>::SharedFuture future);

    void enableTorqueCallback(rclcpp::Client<TriggerSrv>::SharedFuture future);

    void sequenceGoalResponseCallback(const SequenceClientGoalHandle::SharedPtr & future);
    
    void sequenceFeedbackCallback(
        SequenceClientGoalHandle::SharedPtr handle,
        const std::shared_ptr<const MoveGroupSequenceAction::Feedback> feedback
    );
    
    void sequenceResultCallback(const SequenceClientGoalHandle::WrappedResult & result);

    // ROS 2 components
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Clock m_clock;

    rclcpp::CallbackGroup::SharedPtr m_serviceCallbackGroup;

    // Call the servo torque service
    rclcpp::Client<TriggerSrv>::SharedPtr m_enableTorqueClient;
    rclcpp::Client<TriggerSrv>::SharedPtr m_disableTorqueClient;

    // Read joint state to save position
    rclcpp::Subscription<JointStateMsg>::SharedPtr m_jointStateSub;

    // Action client to send a motion sequence
    rclcpp_action::Client<MoveGroupSequenceAction>::SharedPtr m_moveSequenceActionClient;

    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> m_nodeAbstraction;

    // Button to add pose to the path
    QPushButton *m_addPoseButton;

    // Button to enable/disable torque in the servos
    QPushButton *m_toggleTorqueButton;

    // Button to start/stop the robot 
    QPushButton *m_toggleRobotButton;

    PoseTreeWidget* m_poseListWidget;

    QLabel *m_poseStateLabel;

    // Saves the current state of the robot
    JointStateMsg m_currentJointState;

    // Store the saved poses
    SavedPoses m_savedPoses;

    // Save the order of the poses
    Path m_path;

    bool m_robotConnected = false;
    bool m_torqueEnabled = true;
    bool m_serviceWaiting = false;
};

} // end namespace SOArm

#endif // TCP_RVIZ_PLUGIN_HPP