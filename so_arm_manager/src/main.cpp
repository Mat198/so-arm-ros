#include "rclcpp/rclcpp.hpp"
#include "so_arm_manager/manager.hpp"

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SOArm::Manager>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
