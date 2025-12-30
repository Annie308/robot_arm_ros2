#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <functional>
#include <future>
#include <sstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float32.hpp"

#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "robot_arm_interfaces/srv/set_claw.hpp"
#include "robot_arm_interfaces/action/set_target.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

using JointAngles = robot_arm_interfaces::msg::JointAngles;
using SendPos = robot_arm_interfaces::action::SetTarget;
using GoalHandleAngles = rclcpp_action::ServerGoalHandle<SendPos>;
using SetClaw = robot_arm_interfaces::srv::SetClaw;

//controls the servo motors and sends data for forward kinematics


class MotorController : public rclcpp::Node
{
public:


  MotorController(const rclcpp::NodeOptions & options= rclcpp::NodeOptions())
  : Node("motor_controller", options)
  {
    
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid, [[maybe_unused]]std::shared_ptr<const SendPos::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request.");
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
   
	auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleAngles> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleAngles> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
		auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
    std::thread{execute_in_thread}.detach();
    };

    this->action_server_ = rclcpp_action::create_server<SendPos>(
      this,
      "configure_arm",
      handle_goal,
      handle_cancel,
      handle_accepted);

    RCLCPP_INFO(this->get_logger(), "Motors ready.");
      subscription_ptr_ = this->create_subscription<JointAngles>(
		"gyro_arm_angles", 10, std::bind(&MotorController::topic_callback, this, _1));

    this->service_ptr_ = this->create_service<SetClaw>(
      "set_claw",
      std::bind(&MotorController::set_claw, this, _1, _2));

    auto timer_callback_lambda = [this](){return std::bind(&MotorController::topic_callback, this, _1);};

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);
  }

private:
 
  rclcpp_action::Server<SendPos>::SharedPtr action_server_;
  rclcpp::Subscription<JointAngles>::SharedPtr subscription_ptr_;
  rclcpp::Service<SetClaw>::SharedPtr service_ptr_;
  
	rclcpp::TimerBase::SharedPtr timer_;

  void set_claw(
    const std::shared_ptr<SetClaw::Request> request,
    std::shared_ptr<SetClaw::Response> response)
  {
    if (request->clamp_claw){
      RCLCPP_INFO(this->get_logger(), "Clamping claw.");
      // Code to clamp the claw
    } else {
      RCLCPP_INFO(this->get_logger(), "Releasing claw.");
      // Code to release the claw
    }
    response->success = true;
  }

  void topic_callback(const JointAngles::SharedPtr msg)
	{
		for (size_t i=0; i<msg->joint_angles.size(); i++){
//			RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg->joint_angles[i]);
		}
    }

  void execute(const std::shared_ptr<GoalHandleAngles> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
  
      // Update sequence
    // Publish feedback --> from the gyro sensor
    auto feedback = std::make_shared<SendPos::Feedback>();
    auto &angles_now = feedback->curr_angles;
    auto result = std::make_shared<SendPos::Result>();

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->angles_reached = angles_now;

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    
     RCLCPP_INFO(this->get_logger(), "Publish arm feedback: ");
      angles_now = {1.0f, 2.0f, 3.0f};

      for (auto angle: angles_now){
        RCLCPP_INFO(this->get_logger(), "%f ", angle);
      }

      goal_handle->publish_feedback(feedback);
          
      loop_rate.sleep();

      // Check if goal is done
      if (rclcpp::ok()) {
        result->angles_reached = angles_now;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Arm angles: ");
        for (auto angle: result->angles_reached){
          RCLCPP_INFO(this->get_logger(), "%lf", angle);
        }
      }
    }
};  


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =std::make_shared<MotorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}