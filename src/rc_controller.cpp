#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <ctime>

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
    FlightControlNode() : Node("flight_control_node")
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
            std::cout << "Enter control mode (0=attitude, 1=velocity body, 2=velocity ground abs yaw, 3=velocity ground yaw rate, q=quit): ";
            char input;
            std::cin >> input;

            if (input == 'q')
            {
                rclcpp::shutdown();
            }

            uint8_t control_mode = 0;
            switch (input)
            {
            case '0':
            { // Attitude Control
                control_mode = (FCU_VERTICAL::THRUST |
                                FCU_HORIZONTAL::ANGLE |
                                FCU_YAW::RATE |
                                FCU_FRAME::BODY |
                                FCU_STABLE::ENABLE);
                break;
            }
            case '1':
            { // Velocity Body
                control_mode = (FCU_VERTICAL::VELOCITY |
                                FCU_HORIZONTAL::VELOCITY |
                                FCU_YAW::ANGLE |
                                FCU_FRAME::BODY |
                                FCU_STABLE::ENABLE);
                break;
            }
            case '2':
            { // Velocity Ground Abs Yaw
                control_mode = (FCU_VERTICAL::VELOCITY |
                                FCU_HORIZONTAL::VELOCITY |
                                FCU_YAW::ANGLE |
                                FCU_FRAME::GROUND |
                                FCU_STABLE::ENABLE);
                break;
            }
            case '3':
            { // Velocity Ground Yaw Rate
                control_mode = (FCU_VERTICAL::VELOCITY |
                                FCU_HORIZONTAL::VELOCITY |
                                FCU_YAW::RATE |
                                FCU_FRAME::GROUND |
                                FCU_STABLE::ENABLE);
                break;
            }
            default:
                std::cout << "Invalid input. Please try again." << std::endl;
                continue;
            }

            // Update the control mode in joy_msg
            {
                std::lock_guard<std::mutex> lock(joy_msg_mutex_);
                joy_msg->axes[4] = control_mode;
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
