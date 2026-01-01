#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <sstream>
#include <iostream>
#include <optional>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "robot_arm_interfaces/action/set_target.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"
#include "robot_arm_interfaces/msg/joint_positions.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "robot_arm_interfaces/srv/set_claw.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

using JointPositions = robot_arm_interfaces::msg::JointPositions;
using SetTarget = robot_arm_interfaces::action::SetTarget;
using GoalHandleTarget = rclcpp_action::ClientGoalHandle<SetTarget>;
using InverseKin = robot_arm_interfaces::srv::InverseKin;
using SetClaw = robot_arm_interfaces::srv::SetClaw;

enum class GoalType {SET_ARM_POS, SET_WRIST_ROTATION, CLAMP_CLAW, RELEASE_CLAW};

class SetParameter : public rclcpp::Node
{
public:
    SetParameter(const rclcpp::NodeOptions & options)
        : Node("set_param_node", options)
    {
        // declare parameter
        this->declare_parameter("goal_type", 1);

        // timer to periodically read parameter
        timer_ = this->create_wall_timer(1000ms, [this]() {
            int goal_type_param = this->get_parameter("goal_type").as_int();
            RCLCPP_INFO(this->get_logger(), "Current Parameter: %d", goal_type_param);
        });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

class ServiceClient : public rclcpp::Node{
public:
	 ServiceClient(const rclcpp::NodeOptions & options, geometry_msgs::msg::Vector3 target_, GoalType goal_type_) 
    : Node("service_client", options),  target(target_), goal_type(goal_type_)
  {	
		angles_client_ptr_ =this->create_client<InverseKin>("target_angles");
		claw_client_ptr_ =this->create_client<SetClaw>("set_claw");	
	}

	std::optional<std::vector<float>> get_angles(){

		auto request = std::make_shared<InverseKin::Request>();
		request->target.x = target.x;
		request->target.y = target.y;
		request->target.z = target.z;
		request->mode = static_cast<int>(goal_type);

		RCLCPP_INFO(this->get_logger(), "Sending inverse kin service request: %lf, %lf, %lf", 
		request->target.x,  request->target.y, request->target.z);
		
		
        while (!angles_client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return std::nullopt;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        //send the request and execute the callback when the response is received. store in a shared_future
		auto future_result = angles_client_ptr_->async_send_request(request);

		// Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {  
			auto response = future_result.get()->angles;
           
			RCLCPP_INFO(this->get_logger(), "Inverse kinematics response received: ");
			for (auto angle: response){
				RCLCPP_INFO(this->get_logger(), "%f ", angle);
			}
			return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_angles");
        }
		
		return std::nullopt;
    }

	void send_claw_request(){

		if (!this->claw_client_ptr_) {
			RCLCPP_ERROR(this->get_logger(), "Claw service client not initialized");
			return;
		}
		auto request = std::make_shared<SetClaw::Request>();
		
		if (goal_type == GoalType::CLAMP_CLAW){
			RCLCPP_INFO(this->get_logger(), "Request to clamp claw sent.");
			request->clamp_claw = true;
		}else if (goal_type == GoalType::RELEASE_CLAW){
			RCLCPP_INFO(this->get_logger(), "Request to release claw sent.");
			request->clamp_claw = false;
		}else{
			RCLCPP_ERROR(this->get_logger(), "Invalid claw command");
			return;
		}

		while (!claw_client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

       auto result = claw_client_ptr_->async_send_request(request);
    
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {  
			bool response = result.get()->success;
           
			if (response){
				RCLCPP_INFO(this->get_logger(), "claw state set successfully");
			}else{
				RCLCPP_ERROR(this->get_logger(), "Failed to set claw state");
			}
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service set_claw");
        }
	}
	private:
		rclcpp::Client<InverseKin>::SharedPtr angles_client_ptr_;
		rclcpp::Client<SetClaw>::SharedPtr claw_client_ptr_;
		rclcpp::TimerBase::SharedPtr timer_;
		geometry_msgs::msg::Vector3 target;
		GoalType goal_type;
};

//sends the target goal to the motors and is subsrcibed to the gyro sensors to get the current angles
class SetTargetClient : public rclcpp::Node{
public:
	
  SetTargetClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  	: Node("set_target_client", options)
  {	
	target = get_user_input();
		
	if (goal_type == GoalType::SET_ARM_POS || goal_type == GoalType::SET_WRIST_ROTATION){

		//to set joint angles goal to motors
		this->client_ptr_ = rclcpp_action::create_client<SetTarget>(
		this,
		"configure_arm");

		auto timer_callback_lambda = [this](){return send_goal();};
		timer_ = this->create_wall_timer(500ms, timer_callback_lambda);
	}else{
		//to send service request to set claw state (could be an action but idk how the feedback would work :()

		//create client
		auto claw_client = std::make_shared<ServiceClient>(rclcpp::NodeOptions(), *target, goal_type);
		claw_client->send_claw_request();
	}
	
	}
private:
	rclcpp_action::Client<SetTarget>::SharedPtr client_ptr_;

	rclcpp::TimerBase::SharedPtr timer_;

	std::optional<std::vector<float>> goal_angles;
	std::optional<geometry_msgs::msg::Vector3> target;

	GoalType goal_type;
	int goal_type_param;

	void send_goal(){
		//only send goal once
		timer_->cancel();

		if (!target.has_value()){
			RCLCPP_ERROR(this->get_logger(), "No valid target provided, shutting down.");
			rclcpp::shutdown();
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Target received: x: %f, y: %f, z: %f --> sending to inverse kinematics server.", 
			target->x, target->y, target->z);

		//fecthes angles by calling the inverse kinematics service
		//to get joint angles from inverse kinematics server			(this looks kinda bad i shall fix later maybe)

		using namespace std::placeholders;

		//if time out
		if (!this->client_ptr_->wait_for_action_server(5s)) {
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
		}

		auto angles_client = std::make_shared<ServiceClient>(rclcpp::NodeOptions(), *target, goal_type);
		goal_angles = angles_client->get_angles();

		if (!goal_angles.has_value()){
			RCLCPP_ERROR(this->get_logger(), "No goal angles recieved, shutting down.");
			rclcpp::shutdown();
			return;
		}

		auto goal_msg = SetTarget::Goal();

        goal_msg.target_angles = *goal_angles;

		RCLCPP_INFO(this->get_logger(), "Sending target angles goal: ");

		for (auto angle: goal_msg.target_angles){
			RCLCPP_INFO(this->get_logger(), "%f ", angle);
		}

		//configure what to do after sending the goal. Is sent to ROS to be managed later
		auto send_goal_options = rclcpp_action::Client<SetTarget>::SendGoalOptions();

		//when the goal is sent to the server and the server responds, ROS will fill the shared ptr of type
		//rclcpp_action::ClientGoalHandle
		send_goal_options.goal_response_callback = [this](const GoalHandleTarget::SharedPtr & goal_handle)
		{
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			} else {
				RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
			}
		};

		//specify which goal this feedback is for, and also the feedback message
		send_goal_options.feedback_callback = [this](
		GoalHandleTarget::SharedPtr,
		const std::shared_ptr<const SetTarget::Feedback> feedback)
		{
			auto angles_now = feedback->curr_angles;
			RCLCPP_INFO(this->get_logger(), "Current Position Recieved:"); 
			for (size_t i=0; i< angles_now.size(); i++){
				RCLCPP_INFO(this->get_logger(), "%f ", angles_now[i]);
			}
		};

		//handle the response depending on the type of response
		send_goal_options.result_callback = [this](const GoalHandleTarget::WrappedResult & result)
		{
		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
				return;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
				return;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				return;
			}

			RCLCPP_INFO(this->get_logger(), "Result received:");
			
			for (auto res: result.result->angles_reached){
				RCLCPP_INFO(this->get_logger(), "%f ", res);
			}

			rclcpp::shutdown();
		};
		
		this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
	}

	std::optional<geometry_msgs::msg::Vector3> get_user_input()
	{	
		std::string set_goal_type;
		geometry_msgs::msg::Vector3 target_;

		RCLCPP_INFO(this->get_logger(),
			"Enter separated by spaces:\n"
			"(1) Set target arm pos\n"
			"(2) Set target wrist rotation\n"
			"(3) Clamp claw\n"
			"(4) Release claw >> ");
		
		std::getline(std::cin, set_goal_type);  // read the whole line, including spaces
		std::stringstream str_set_goal_type(set_goal_type);     // put it into a stringstream
		char choice;

		if (!(str_set_goal_type >> choice)){
			RCLCPP_ERROR(this->get_logger(), "Invalid input");
			return std::nullopt;
		}     
		switch(static_cast<int>(choice)){
			case '1':
				RCLCPP_INFO(this->get_logger(), "Request received: Set target arm pos");
				goal_type = GoalType::SET_ARM_POS;
				break;
			case '2':
				RCLCPP_INFO(this->get_logger(), "Request received: Set target wrist rotation");
				goal_type = GoalType::SET_WRIST_ROTATION;
				break;
			case '3':
				RCLCPP_INFO(this->get_logger(), "Request received: Clamp claw");
				goal_type = GoalType::CLAMP_CLAW;
				break;
			case '4':
				RCLCPP_INFO(this->get_logger(), "Request received: Release claw");
				goal_type = GoalType::RELEASE_CLAW;
				break;
			default:
				RCLCPP_ERROR(this->get_logger(),"Request received: Invalid input");
				return std::nullopt;
		}

		if (goal_type== GoalType::SET_ARM_POS){
			RCLCPP_INFO(this->get_logger(), "Enter target arm position as x y z >> ");
		}
		else if (goal_type== GoalType::SET_WRIST_ROTATION){
			RCLCPP_INFO(this->get_logger(), "Enter target wrist rotation as x y z >> ");
		}
		else{
			return std::nullopt;
		}
		
		std::string str_target;
		std::getline(std::cin, str_target);  // read the whole line, including spaces
		std::stringstream ss(str_target);     // put it into a stringstream

		float x, y, z;

		if (!(ss >> x >> y >> z)){
			RCLCPP_ERROR(this->get_logger(), "Invalid target input");
			return std::nullopt;
		}else{
			target_.x =x;
			target_.y =y;
			target_.z =	z;
		}
		RCLCPP_INFO(this->get_logger(), "You entered: %f %f %f", x, y, z);
			return target_;
	}
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	//rclcpp::spin(std::make_shared<SetParameter>(rclcpp::NodeOptions()));
	rclcpp::spin(std::make_shared<SetTargetClient>());
	rclcpp::shutdown();
	return 0;
}