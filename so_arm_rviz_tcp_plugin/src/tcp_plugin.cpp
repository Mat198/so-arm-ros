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

    m_poseListWidget = new PoseTreeWidget();
    m_poseListWidget->setItemDelegateForColumn(0, new PoseDelegate(this));
    m_poseListWidget->setItemDelegateForColumn(1, new PoseDelegate(this));
    planningFrameLayout->addWidget(m_poseListWidget);

    connect(m_poseListWidget, &PoseTreeWidget::itemDropped, this, &RobotTcp::onPoseMoved);
    connect(m_poseListWidget, &QTreeWidget::itemChanged, this, &RobotTcp::onPoseChanged);

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
    connect(this, &RobotTcp::updateButtonText, m_toggleTorqueButton, &QPushButton::setText);

    connect(m_toggleRobotButton, &QPushButton::clicked, this, &RobotTcp::toggleRobot);
    connect(m_addPoseButton, &QPushButton::clicked, this, &RobotTcp::addPose);

    mainLayout->addWidget(controlFrameGroup);
}

void RobotTcp::onInitialize()  {

    using std::placeholders::_1;
    using std::placeholders::_2;
    
    m_nodeAbstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = m_nodeAbstraction->get_raw_node();

    m_serviceCallbackGroup = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_jointStateSub = m_node->create_subscription<JointStateMsg>(
        "joint_states", 10, std::bind(&RobotTcp::jointStatesCallback, this, _1));
    
    m_enableTorqueClient = m_node->create_client<TriggerSrv>(
        "so_arm/enable_torque", rmw_qos_profile_services_default, m_serviceCallbackGroup);
    m_disableTorqueClient = m_node->create_client<TriggerSrv>(
        "so_arm/disable_torque", rmw_qos_profile_services_default, m_serviceCallbackGroup);
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
    m_path.push_back(poseDefaultName);

    // Creates the UI pose element
    createPoseItem(poseDefaultName, m_currentJointState.position);
    
    RCLCPP_INFO_STREAM(m_node->get_logger(), 
        "Saved pose " << poseDefaultName << ": " << print::vector2Str(m_currentJointState.position)
    );

    RCLCPP_INFO_STREAM(m_node->get_logger(), "Saved poses: " << print::map2Str(m_savedPoses));
}


void RobotTcp::createPoseItem(const std::string &poseDefaultName, const std::vector<double> &pose) {
    
    // Blocking signal during manual edit
    m_poseListWidget->blockSignals(true);

    QTreeWidgetItem* poseItem = new QTreeWidgetItem(m_poseListWidget);
    // Saving data to verify if new name is valid and then set the previous if not.
    poseItem->setText(0, poseDefaultName.c_str());
    poseItem->setData(0, Qt::UserRole, poseDefaultName.c_str());
    poseItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsDragEnabled);

    QTreeWidgetItem* detailItem = new QTreeWidgetItem(poseItem);
    detailItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemNeverHasChildren);
    detailItem->setText(0, "Joint positions:");
    // Saving data to verify if new name is valid and then set the previous if not.
    detailItem->setText(1, print::vector2Str(pose).c_str());
    detailItem->setData(1, Qt::UserRole, print::vector2Str(pose).c_str());
    // TODO: Add widget with cartesian position

    poseItem->setExpanded(true);
    m_poseListWidget->resizeColumnToContents(0);
    m_poseListWidget->blockSignals(false);
}

void RobotTcp::disableTorqueCallback(rclcpp::Client<TriggerSrv>::SharedFuture future) {
    
    auto response = future.get();
    m_serviceWaiting = false;

    if (!response->success) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to disable torque.");
        return;
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "Torque disabled!");
    m_torqueEnabled = false;
    Q_EMIT updateButtonText("Enable Torque");
}

void RobotTcp::enableTorqueCallback(rclcpp::Client<TriggerSrv>::SharedFuture future) {

    auto response = future.get();
    m_serviceWaiting = false;

    if (!response->success) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to enable torque.");
        return;
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "Torque enabled!");
    m_torqueEnabled = true;
    Q_EMIT updateButtonText("Disable Torque");
}

void RobotTcp::toggleTorque() {
    
    if (m_serviceWaiting) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Still waiting for a enable/disable response!");
        return;
    }

    const auto req = std::make_shared<TriggerSrv::Request>();
    using std::placeholders::_1;

    if (m_torqueEnabled) {
        m_serviceWaiting = true;
        auto result = m_disableTorqueClient->async_send_request(
            req, std::bind(&RobotTcp::disableTorqueCallback, this, _1));
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Torque disable request sent!");
        return;
    }

    auto result = m_enableTorqueClient->async_send_request(
        req, std::bind(&RobotTcp::enableTorqueCallback, this, _1));
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Torque enable request sent!");
    m_torqueEnabled = true;
}

void RobotTcp::toggleRobot() {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Clicked toggle robot button!");
}

void RobotTcp::onPoseMoved() {

    RCLCPP_INFO_STREAM(m_node->get_logger(), "On pose moved called!");

    m_path = Path();
    m_path.reserve(m_poseListWidget->topLevelItemCount());

    for (int i = 0; i < m_poseListWidget->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = m_poseListWidget->topLevelItem(i);
        m_path.push_back(item->text(0).toStdString());
    }

    RCLCPP_INFO_STREAM(m_node->get_logger(), "Updated pose list: " << print::vector2Str(m_path));
}

void RobotTcp::onPoseChanged(QTreeWidgetItem *poseItem, int column) {

    // Name of the pose update
    if (column == 0 && poseItem->parent() == nullptr) {
        updatePoseName(poseItem);
    }
    
    // Update on the pose value
    if (column == 1) {
        // TODO: Update the pose value on the saved pose map
        updatePoseValues(poseItem);
    }
}

void RobotTcp::updatePoseName(QTreeWidgetItem *poseItem) {
    
    const std::string oldName = poseItem->data(0, Qt::UserRole).toString().toStdString();
    const std::string newName = poseItem->text(0).toStdString();

    if (newName.empty()) {
        poseItem->setText(0, oldName.c_str());
        std::string errorMsg = "Failed to rename pose " + oldName + " to " + newName + 
            ". Empty name is forbideen.";
        m_poseStateLabel->setText(errorMsg.c_str());
        return;
    }

    if (m_savedPoses.find(newName) != m_savedPoses.end()) {
        poseItem->setText(0, oldName.c_str());
        std::string errorMsg = "Failed to rename pose " + oldName + " to " + newName + 
            ". The pose " + newName + " already exists.";
        m_poseStateLabel->setText(errorMsg.c_str());
        return;
    }
    
    std::vector<double> poseValue = m_savedPoses.at(oldName);
    m_savedPoses.insert_or_assign(newName, poseValue);
    m_savedPoses.erase(oldName);
    std::replace(m_path.begin(), m_path.end(), oldName, newName);

    RCLCPP_INFO_STREAM(
        m_node->get_logger(), "Changed pose " << oldName << " to " << newName << "!");

}

void RobotTcp::updatePoseValues(QTreeWidgetItem *poseValues) {

    const std::string oldPose = poseValues->data(1, Qt::UserRole).toString().toStdString();
    const std::string newPose = poseValues->text(1).toStdString();
    // TODO: Update pose value
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
