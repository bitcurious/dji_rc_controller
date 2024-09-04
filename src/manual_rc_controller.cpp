#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <ctime>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

namespace FCU_VERTICAL
{
    enum
    {
        MASK = 0x30,
        VELOCITY = 0x00,
        POSITION = 0x10,
        THRUST = 0x20
    };
};

namespace FCU_HORIZONTAL
{
    enum
    {
        MASK = 0xC0,
        ANGLE = 0x00,
        VELOCITY = 0x40,
        POSITION = 0x80,
        ANGULAR_RATE = 0xC0
    };
};

namespace FCU_YAW
{
    enum
    {
        MASK = 0x08,
        ANGLE = 0x00,
        RATE = 0x08
    };
};

namespace FCU_FRAME
{
    enum
    {
        MASK = 0x06,
        GROUND = 0x00,
        BODY = 0x02
    };
};

namespace FCU_STABLE
{
    enum
    {
        MASK = 0x01,
        DISABLE = 0x00,
        ENABLE = 0x01
    };
};

class FlightControlNode : public rclcpp::Node
{
public:
    FlightControlNode() : Node("manual_flight_control_node")
    {
        rc_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(
            "/wrapper/psdk_ros2/flight_control_setpoint_generic", 10);
        rc_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/wrapper/psdk_ros2/rc", 10, std::bind(&FlightControlNode::rcCallback, this, std::placeholders::_1));

        joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
        joy_msg->axes.resize(5);

        timer_ = this->create_wall_timer(500ms, std::bind(&FlightControlNode::timerCallback, this));

        std::thread(&FlightControlNode::userInputThread, this).detach();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr rc_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rc_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Joy::SharedPtr joy_msg;
    std::mutex joy_msg_mutex_; // Mutex to protect shared data

    void rcCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_msg_mutex_);
        joy_msg->axes[0] = msg->axes[0];
        joy_msg->axes[1] = msg->axes[1];
        joy_msg->axes[2] = msg->axes[2];
        joy_msg->axes[3] = msg->axes[3];
    }

    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(joy_msg_mutex_);
        rc_publisher_->publish(*joy_msg);
        // RCLCPP_INFO(this->get_logger(), "Published RC data with control mode: %d", static_cast<int>(joy_msg->axes[4]));
    }

    void userInputThread()
    {
        while (rclcpp::ok())
        {
            uint8_t control_mode = 0;

            // Frame control mode selection
            std::cout << "Enter frame control mode (1=ground, 2=body, q=quit): ";
            char f_input;
            std::cin >> f_input;

            if (f_input == 'q')
            {
                rclcpp::shutdown();
                return;
            }

            switch (f_input)
            {
            case '1':
                control_mode |= FCU_FRAME::GROUND;
                break;
            case '2':
                control_mode |= FCU_FRAME::BODY;
                break;
            default:
                std::cout << "Invalid input. Please try again." << std::endl;
                continue;
            }

            // Horizontal mode selection
            std::cout << "Enter horizontal control mode (1=Angle, 2=Velocity, 3=Position, q=quit): ";
            char h_input;
            std::cin >> h_input;
            if (h_input == 'q')
            {
                rclcpp::shutdown();
                return;
            }

            switch (h_input)
            {
            case '1':
                control_mode |= FCU_HORIZONTAL::ANGLE;
                break;
            case '2':
                control_mode |= FCU_HORIZONTAL::VELOCITY;
                break;
            case '3':
                control_mode |= FCU_HORIZONTAL::POSITION;
                break;
            default:
                std::cout << "Invalid input. Please try again." << std::endl;
                continue;
            }

            // Vertical control mode selection
            std::cout << "Enter vertical control mode (1=Velocity, 2=Position, 3=Thrust, q=quit): ";
            char v_input;
            std::cin >> v_input;
            if (v_input == 'q')
            {
                rclcpp::shutdown();
                return;
            }

            switch (v_input)
            {
            case '1':
                control_mode |= FCU_VERTICAL::VELOCITY;
                break;
            case '2':
                control_mode |= FCU_VERTICAL::POSITION;
                break;
            case '3':
                control_mode |= FCU_VERTICAL::THRUST;
                break;
            default:
                std::cout << "Invalid input. Please try again." << std::endl;
                continue;
            }

            // Yaw control mode selection
            std::cout << "Enter yaw control mode (1=Angle, 2=Rate, q=quit): ";
            char y_input;
            std::cin >> y_input;

            if (y_input == 'q')
            {
                rclcpp::shutdown();
                return;
            }

            switch (y_input)
            {
            case '1':
                control_mode |= FCU_YAW::ANGLE;
                break;
            case '2':
                control_mode |= FCU_YAW::RATE;
                break;
            default:
                std::cout << "Invalid input. Please try again." << std::endl;
                continue;
            }

            // Update the control mode in joy_msg
            {
                std::lock_guard<std::mutex> lock(joy_msg_mutex_);
                joy_msg->axes[4] = control_mode;
                std::cout << "Control mode set to: " << static_cast<int>(control_mode) << std::endl;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightControlNode>());
    rclcpp::shutdown();
    return 0;
}
