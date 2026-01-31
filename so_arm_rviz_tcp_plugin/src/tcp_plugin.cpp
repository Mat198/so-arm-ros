#include "so_arm_rviz_tcp_plugin/tcp_plugin.hpp"

namespace SOArm {

RobotTcp::RobotTcp(QWidget *parent) :
    rviz_common::Panel(parent)
{
    setupGui();
}

RobotTcp::~RobotTcp() {
    delete m_toggleTorqueButton;
    delete m_toggleRobotButton;
    delete m_addPoseButton;
};

void RobotTcp::setupGui() {

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Adding robot status
    QGroupBox* statusFrameGroup = new QGroupBox("Status frame", this);
    mainLayout->addWidget(statusFrameGroup);

    // Configuring the planning frame
    QGroupBox* planningFrameGroup = new QGroupBox("Planning frame", this);
    QFormLayout* planningFrameLayout = new QFormLayout(planningFrameGroup);
    
    m_poseStateLabel = new QLabel();
    m_poseStateLabel->setText("Robot not connected");
    planningFrameLayout->addWidget(m_poseStateLabel);

    m_poseListWidget = new QTreeWidget();
    m_poseListWidget->setHeaderHidden(true);
    m_poseListWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_poseListWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    planningFrameLayout->addWidget(m_poseListWidget);

    mainLayout->addWidget(planningFrameGroup);

    // Configuring control frame
    QGroupBox* controlFrameGroup = new QGroupBox("Control frame", this);
    QFormLayout* controlFrameLayout = new QFormLayout(controlFrameGroup);

    // Starting with the robot locked but not started
    m_toggleTorqueButton = new QPushButton("Disable Torque");
    m_toggleRobotButton = new QPushButton("Start");
    m_addPoseButton = new QPushButton("Add Pose");

    controlFrameLayout->addWidget(m_toggleTorqueButton);
    controlFrameLayout->addWidget(m_toggleRobotButton);
    controlFrameLayout->addWidget(m_addPoseButton);

    // Connection the buttons to their functionality
    connect(m_toggleTorqueButton, &QPushButton::clicked, this, &RobotTcp::toggleTorque);
    connect(m_toggleRobotButton, &QPushButton::clicked, this, &RobotTcp::toggleRobot);
    connect(m_addPoseButton, &QPushButton::clicked, this, &RobotTcp::addPose);

    mainLayout->addWidget(controlFrameGroup);
}

void RobotTcp::onInitialize()  {
    
    m_nodeAbstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = m_nodeAbstraction->get_raw_node();
    using std::placeholders::_1;
    m_jointStateSub = m_node->create_subscription<JointStateMsg>(
        "joint_states", 10, std::bind(&RobotTcp::jointStatesCallback, this, _1));
}

void RobotTcp::save(rviz_common::Config config) const {
    
    rviz_common::Panel::save(config);
}

void RobotTcp::load(const rviz_common::Config& config) {
    
    rviz_common::Panel::load(config);
}

void RobotTcp::addPose() {

    if (m_currentJointState.position.empty()) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Current Joint Pose is empty!");
        m_poseStateLabel->setText("Failed to add pose. Robot disconnected.");
        return;
    }

    std::string poseDefaultName = "Pose " + std::to_string(m_savedPoses.size());
    m_savedPoses[poseDefaultName] = m_currentJointState.position;

    createPoseItem(poseDefaultName, m_currentJointState.position);
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), 
        "Saved pose " << poseDefaultName << ": " << print::vector2Str(m_currentJointState.position)
    );
}


void RobotTcp::createPoseItem(const std::string &poseDefaultName, const std::vector<double> &pose) {
    
    QTreeWidgetItem* poseItem = new QTreeWidgetItem(m_poseListWidget);
    poseItem->setText(0, poseDefaultName.c_str());
    poseItem->setFlags(
        poseItem->flags() | Qt::ItemIsEditable | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled
    );

    QTreeWidgetItem* detailItem = new QTreeWidgetItem(poseItem);
    detailItem->setFlags(Qt::ItemIsEditable);
    
    QWidget* detailWidget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(detailWidget);
    const std::string jointPose = "Joint positions:" + print::vector2Str(pose);
    layout->addWidget(new QLabel(jointPose.c_str()));
    layout->setContentsMargins(5, 5, 5, 5);

    m_poseListWidget->setItemWidget(detailItem, 0, detailWidget);
    
    poseItem->setExpanded(true);
}

void RobotTcp::toggleTorque() {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Clicked toggle torque button!");

}

void RobotTcp::toggleRobot() {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Clicked toggle robot button!");
}

void RobotTcp::jointStatesCallback(const JointStateMsg::SharedPtr msg) {
    m_currentJointState = *msg;
    if (!m_robotConnected) {
        m_robotConnected = true;
        // TODO: Implementar rotina de desconexão
    }
}

} // namespace SOArm

#include <pluginlib/class_list_macros.hpp>

// Macro que registra este plugin na lista de plugins do RVIZ
PLUGINLIB_EXPORT_CLASS(SOArm::RobotTcp, rviz_common::Panel)
