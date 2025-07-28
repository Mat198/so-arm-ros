// Keyboard includes
#include <termios.h>
#include <unistd.h>

// ROS2 client library
#include <rclcpp/rclcpp.hpp>

// ROS2 msg includes
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const char ESC = '\x1b';
const char EXIT = '\x03';
const size_t STEPS_PER_REVOLUTION = 4096;
const size_t MAX_STEPS_PER_SECOND = 3000;
const double MAX_SPEED = 2.0 * M_PI * MAX_STEPS_PER_SECOND / STEPS_PER_REVOLUTION;
const double FREQUENCY = 50; // Hz

struct KeyAction {
    int joint;
    int direction;
};


const std::unordered_map<char, KeyAction> BINDINGS = {
    {'q', KeyAction{0, 1}}, {'a', KeyAction{0, -1}},
    {'w', KeyAction{1, 1}}, {'s', KeyAction{1, -1}},
    {'e', KeyAction{2, 1}}, {'d', KeyAction{2, -1}},
    {'r', KeyAction{3, 1}}, {'f', KeyAction{3, -1}},
    {'t', KeyAction{4, 1}}, {'g', KeyAction{4, -1}},
    {'y', KeyAction{5, 1}}, {'h', KeyAction{5, -1}},
    {'z', KeyAction{-1, 1}}, {'c', KeyAction{-1, -1}}
};

using namespace std::placeholders;

class KeyboardReader final {
public:
    KeyboardReader(){
        // get the console in raw mode
        if (tcgetattr(0, &m_cooked) < 0){
            throw std::runtime_error("Failed to get old console mode");
        }
        struct termios raw;
        memcpy(&raw, &m_cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        raw.c_cc[VTIME] = 1;
        raw.c_cc[VMIN] = 0;
        if (tcsetattr(0, TCSANOW, &raw) < 0){
            RCLCPP_FATAL_STREAM(m_logger, "Failed to set new console mode");
            tcsetattr(0, TCSANOW, &m_cooked);
            rclcpp::shutdown();
        }
    }

    char readOne(){
        char c = 0;
        int rc = read(0, &c, 1);
        if (rc < 0){
            RCLCPP_FATAL_STREAM(m_logger, "Failed to read key");
            tcsetattr(0, TCSANOW, &m_cooked);
            rclcpp::shutdown();
        }
        return c;
    }

    ~KeyboardReader(){
        RCLCPP_INFO_STREAM(m_logger, "Restoring terminal...");
        tcsetattr(0, TCSANOW, &m_cooked);
        RCLCPP_INFO_STREAM(m_logger, "Terminal restored!");
    }

private:
    struct termios m_cooked;
    rclcpp::Logger m_logger = rclcpp::get_logger("keyboard_reader");

};

class ArmKeyboardController: public rclcpp::Node {

private:
    using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    using JointState = sensor_msgs::msg::JointState;

public:

    ArmKeyboardController(): 
        Node("so_arm_keyboard_controller"),
        m_logger(this->get_logger()),
        m_speedScale(0.1),
        m_active(false)
    {
        RCLCPP_INFO_STREAM(m_logger, 
            "\n---------------------------\n"
            "For move joint 0: +q -a\n"
            "For move joint 1: +w -s\n"
            "For move joint 2: +e -d\n"
            "For move joint 3: +r -f\n"
            "For move joint 4: +t -g\n"
            "For move joint 5: +y -h\n"
            "Speed: z/c increase/decrease max speeds by 10%\n"
            "anything else or no key: stop\n"
            "---------------------------\n"
            "CTRL-C to quit\n"
        );
        
        // Initiates the target with empty data
        const auto zeroVector = std::vector<double>(6, 0.0);
        m_target.positions = m_target.accelerations = zeroVector;
        m_target.velocities = std::vector<double>(6, m_speedScale);

        // Initiates the joint state with zeroes
        m_jointState.position = m_jointState.velocity = m_jointState.effort = zeroVector;

        m_targetPublish = this->create_publisher<JointTrajectoryPoint>("/so_arm/target", 10);

        const uint period = static_cast<uint>(round(1/FREQUENCY));
        m_publisherTimer = this->create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&ArmKeyboardController::timerCallback, this)
        );
        
        m_readKeysThread = std::thread(std::bind(&ArmKeyboardController::keysHandler, this));
        this->declare_parameter<bool>("show_inputs", false);

        using std::placeholders::_1;
        m_jointStateSub = this->create_subscription<JointState>(
            "/joint_states", 10, std::bind(&ArmKeyboardController::jointStateCallback, this, _1));
    }

    ~ArmKeyboardController() {
        m_readKeysThread.join();
        m_keysReader.~KeyboardReader();
    }

private:

    void timerCallback() {
        if (!m_active) {
            return;
        }
        m_targetPublish->publish(m_target);
    }

    void jointStateCallback(const JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->position.size(); i++) {
            m_jointState.position[msg->position.size() - i - 1] = msg->position[i];
        }
        if (!m_active) {
            m_target.positions = m_jointState.position;
            m_active = true;
        }
    }

    void keysHandler() {
        char keyPressed;
        while (rclcpp::ok()) {
        // get the next event from the keyboard
            try {
                keyPressed = m_keysReader.readOne();
            }
            catch (const std::runtime_error &exeption) {
                RCLCPP_FATAL_STREAM(
                    m_logger, "Got exeption while reading key: " << exeption.what()
                );
                rclcpp::shutdown();
                return;
            }
            if (keyPressed != '\0' && this->get_parameter("show_inputs").as_bool()){
                RCLCPP_INFO_STREAM(
                    m_logger, "Input = " << keyPressed << ";"
                ); 
            }
            updateTarget(keyPressed);
        }
    }


    void updateTarget(char key) {
        if (BINDINGS.find(key) == BINDINGS.end()) {
            // Stops the robot at current position
            // m_target.positions = m_jointState.position;

            if (key == EXIT || key == ESC) {
                rclcpp::shutdown();
            }
            return;
        }
        const int joint = BINDINGS.at(key).joint;
        const int direction = BINDINGS.at(key).direction;
        if (joint >= 0) {
            m_target.positions[joint] += direction * m_speedScale * MAX_SPEED * (1/FREQUENCY);
            RCLCPP_DEBUG_STREAM(
                m_logger, 
                std::fixed << std::setprecision(3) << "Updating joint "<< joint 
                << " position from " << m_jointState.position[joint] 
                << " to " << m_target.positions[joint]
            );
            // Joint limit is handled by the servo
        } else {
            m_speedScale += direction * 0.1;
            if (m_speedScale <= 0) m_speedScale = 0.001;
            if (m_speedScale > 1) m_speedScale = 1;
            RCLCPP_INFO_STREAM(m_logger, "Speed set to " << m_speedScale * 100 << "%");
        }
        m_target.velocities = std::vector<double>(6, m_speedScale * MAX_SPEED);
    }

    /* === ROS2 Components === */

    rclcpp::Logger m_logger;

    rclcpp::Subscription<JointState>::SharedPtr m_jointStateSub;
    rclcpp::Publisher<JointTrajectoryPoint>::SharedPtr m_targetPublish;
    rclcpp::TimerBase::SharedPtr m_publisherTimer;

    JointTrajectoryPoint m_target;
    JointState m_jointState;

    std::thread m_readKeysThread;

    // TODO: Implement PWM mode to close control loop internally
    // bool m_pwmMode; // PWM mode (true) or Position mode (false)
    double m_speedScale; // in rad/s
    bool m_active; // Active after receive the first joint position. Prevent robot move abruptaly 

    KeyboardReader m_keysReader;
};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<ArmKeyboardController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
