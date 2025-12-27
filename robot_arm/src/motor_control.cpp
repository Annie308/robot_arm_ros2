#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

//controls the servo motors and sends data for forward kinematics
//client
class MotorController : public rclcpp::Node
{
    public:
        MotorController() : Node("motor_controller"), count_(0)
        {
            //publisher_ = this->create_publisher<std_msgs::msg::Float64>("send_angles", 10);
            client_ = this->create_client<robot_arm_interfaces::srv::InverseKin>("arm_angles");
        }
        /*
        void sendAngles(){
            //lamba function that returns nothing
            //Remember: [capture](parameters) -> return_type {body}
            auto timer_callback = 
            [this]() -> void {
                std::vector<double> message = res_angles;
                
                for (size_t i=0; i < message.size(); i++){
                    std_msgs::msg::Float64 msg;
                    msg.data = static_cast<double>(message[i]);
              
                    RCLCPP_INFO(this->get_logger(), "Publishing: '%lf",message[i]);
                    this->publisher_->publish(msg);
                }
            };
            timer_ = this->create_wall_timer(500ms, timer_callback);
        }
            */

        void recieveAngles(float x, float y, float z){
             auto request = std::make_shared<robot_arm_interfaces::srv::InverseKin::Request>();

            request->target.x = x;
            request->target.y = y;
            request->target.z = z;

            while (!client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            auto result = client_->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                res_angles = result.get()->angles;

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recieved %zu angles:", res_angles.size());

                for (size_t i=0; i < res_angles.size(); i++){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle %ld : %lf", i, res_angles[i]);
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service inverse_kin");
            }
        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        rclcpp::Client<robot_arm_interfaces::srv::InverseKin>::SharedPtr client_;
        size_t count_;
        std::vector<double> res_angles;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =std::make_shared<MotorController>();

  node ->recieveAngles(
    std::stod(argv[1]),
    std::stod(argv[2]),
    std::stod(argv[3])
  );

  //node->sendAngles();
rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}