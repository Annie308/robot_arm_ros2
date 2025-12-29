#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <functional>
#include <future>
#include <sstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_arm_interfaces/action/set_target.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using JointAngles = robot_arm_interfaces::msg::JointAngles;

//controls the servo motors and sends data for forward kinematics


class MotorController : public rclcpp::Node
{
public:
  using SendPos = robot_arm_interfaces::action::SetTarget;
  using GoalHandleAngles = rclcpp_action::ServerGoalHandle<SendPos>;

  MotorController(const rclcpp::NodeOptions & options= rclcpp::NodeOptions())
  : Node("motor_controller", options)
  {

      subscription_ptr_ = this->create_subscription<JointAngles>(
		"gyro_arm_angles", 10, std::bind(&MotorController::topic_callback, this, _1));

    auto timer_callback_lambda = [this](){return std::bind(&MotorController::topic_callback, this, _1);};

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);

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
      "set_target",
      handle_goal,
      handle_cancel,
      handle_accepted);
  
  }

private:
 
  rclcpp_action::Server<SendPos>::SharedPtr action_server_;
  rclcpp::Subscription<JointAngles>::SharedPtr subscription_ptr_;
	rclcpp::TimerBase::SharedPtr timer_;

  void topic_callback(const JointAngles::SharedPtr msg)
	{
		for (size_t i=0; i<msg->joint_angles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg->joint_angles[i]);
		}
    }

  void execute(const std::shared_ptr<GoalHandleAngles> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
  
      // Update sequence
    // Publish feedback --> from the gyro sensor
    auto feedback = std::make_shared<SendPos::Feedback>();
    auto &pos_now = feedback->curr_pos;
    auto result = std::make_shared<SendPos::Result>();

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->target_reached = pos_now;

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    
      pos_now.x = 1.0f;
      pos_now.y = 2.0f;
      pos_now.z = 3.0f;

      RCLCPP_INFO(this->get_logger(), "Publish arm feedback: angle 1: %lf\nangle 2:%lf \nangle3: %lf", 
        feedback->curr_pos.x, feedback->curr_pos.y, feedback->curr_pos.z);

      goal_handle->publish_feedback(feedback);
          
      loop_rate.sleep();

      // Check if goal is done
      if (rclcpp::ok()) {
        result->target_reached = pos_now;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Arm angles: ");

        RCLCPP_INFO(this->get_logger(), "%lf, %f, %lf", pos_now.x, pos_now.y, pos_now.z);
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