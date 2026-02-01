#ifndef TCP_RVIZ_PLUGIN_HPP
#define TCP_RVIZ_PLUGIN_HPP

// RVIZ Plugin base
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/display_context.hpp>

// ROS 2 Interfaces
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

namespace SOArm {

using TriggerSrv = std_srvs::srv::Trigger;
using JointStateMsg = sensor_msgs::msg::JointState;

using SavedPoses = std::unordered_map<std::string, std::vector<double>>;
using Path = std::vector<std::string>;

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

private:

    void setupGui();

    void planPath();

    void createPoseItem(const std::string &poseDefaultName, const std::vector<double> &pose);

    void updatePoseName(QTreeWidgetItem *poseItem);

    void updatePoseValues(QTreeWidgetItem *poseString);

    void jointStatesCallback(const JointStateMsg::SharedPtr msg);

    // ROS 2 components
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Clock m_clock;

    // Call the servo torque service
    rclcpp::Service<TriggerSrv>::SharedPtr m_toggleTorqueSrv;

    // Read joint state to save position
    rclcpp::Subscription<JointStateMsg>::SharedPtr m_jointStateSub;

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
};

} // end namespace SOArm

#endif // TCP_RVIZ_PLUGIN_HPP