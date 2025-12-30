#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class SendJontAngles : public rclcpp::Node
{
public:
  SendJontAngles()
  : Node("gyro_sensor"), count_(0)
  {

    publisher_ = this->create_publisher<robot_arm_interfaces::msg::JointAngles>("gyro_arm_angles", 10);
   
    auto timer_callback =
      [this]() -> void {
        auto message = robot_arm_interfaces::msg::JointAngles();
        message.joint_angles.push_back(0.1);
        message.joint_angles.push_back(0.2);
        message.joint_angles.push_back(0.3);
         message.joint_angles.push_back(0.4);
          message.joint_angles.push_back(0.5);
           message.joint_angles.push_back(0.6);

        for (size_t i=0; i < message.joint_angles.size(); i++){
            RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.joint_angles[i]);
        }
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(2000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<robot_arm_interfaces::msg::JointAngles>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendJontAngles>());
  rclcpp::shutdown();
  return 0;
}