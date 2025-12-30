#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <sstream>
#include <iostream>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "robot_arm_interfaces/action/set_target.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"
#include "robot_arm_interfaces/msg/joint_states.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "robot_arm_interfaces/srv/set_claw.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

using JointStates = robot_arm_interfaces::msg::JointStates;
using SetTarget = robot_arm_interfaces::action::SetTarget;
using GoalHandleTarget = rclcpp_action::ClientGoalHandle<SetTarget>;
using InverseKin = robot_arm_interfaces::srv::InverseKin;
using SetClaw = robot_arm_interfaces::srv::SetClaw;

enum class GoalType {SET_ARM_POS, SET_WRIST_ROTATION, CLAMP_CLAW, RELEASE_CLAW};


//sends a service to the inverse kin server to receive neccessary joint angles to reach target position
class GetAnglesClient: public rclcpp::Node
{
public:
    GetAnglesClient(const rclcpp::NodeOptions & options, geometry_msgs::msg::Vector3 target_, GoalType goal_type_) 
    : Node("get_angles_client", options),  target(target_), goal_type(goal_type_)
    {
        client_ptr_ =this->create_client<InverseKin>("target_arm_angles");
		claw_client_ptr_ =this->create_client<SetClaw>("set_claw");	
    }

    std::vector<float> get_angles(){

        auto request = std::make_shared<InverseKin::Request>();
        
        request->target.x = target.x;
        request->target.y = target.y;
        request->target.z = target.z;
		request->mode = static_cast<int>(goal_type);

        RCLCPP_INFO(this->get_logger(), "Send inverse kin service request: %lf, %lf, %lf", 
            request->target.x,  request->target.y, request->target.z);

        while (!client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return {};
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

       auto result = client_ptr_->async_send_request(request);
    
        // Wait for the result.
        //shared_from_this is a smart pointer that can point to this
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {  
			std::vector<float> response = result.get()->angles;
           
            RCLCPP_INFO(this->get_logger(), "Angles recieved from inverse kin:");
            for (auto angle : response){
                RCLCPP_INFO(this->get_logger(), "%f ", angle);
            }
			return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service arm_angles");
        }

		return {};
    }

	void set_claw(){
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
        //shared_from_this is a smart pointer that can point to this
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
    rclcpp::Client<InverseKin>::SharedPtr client_ptr_;
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
	//a client needs 3 things:
	/*
	1.The templated action type name: SetTargetClient.
	2.A ROS 2 node to add the action client to: this.
	3. The action name: 'set_target'.
	*/
	target = get_user_input();

	if (goal_type == GoalType::SET_ARM_POS || goal_type == GoalType::SET_WRIST_ROTATION){
		this->target_client_ptr_ = rclcpp_action::create_client<SetTarget>(
		this,
		"configure_arm");
	}else{
		auto set_claw_client = std::make_shared<GetAnglesClient>(rclcpp::NodeOptions(), *target, goal_type);
		set_claw_client->set_claw();
	}
	auto timer_callback_lambda = [this](){return send_goal();};
	timer_ = this->create_wall_timer(500ms, timer_callback_lambda);
	}
private:
	rclcpp_action::Client<SetTarget>::SharedPtr target_client_ptr_ = nullptr;
	
	std::vector<float> goal_angles;
	rclcpp::TimerBase::SharedPtr timer_;
	std::optional<geometry_msgs::msg::Vector3> target;
	GoalType goal_type;

	void send_goal(){

		if (goal_type != GoalType::SET_ARM_POS && goal_type != GoalType::SET_WRIST_ROTATION){
			rclcpp::shutdown();			//this is rlly bad but idk how else to stop it from spinning...
			return;
		}

		if (!target.has_value()){
			RCLCPP_ERROR(this->get_logger(), "No valid target provided, shutting down.");
			rclcpp::shutdown();
			return;
		}
		if (!this->target_client_ptr_) {
			RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
			rclcpp::shutdown();
			return;
		}

		timer_->cancel();

		using namespace std::placeholders;

		auto get_angles_client = std::make_shared<GetAnglesClient>(rclcpp::NodeOptions(), *target, goal_type);
		get_angles_client->get_angles();

		RCLCPP_INFO(this->get_logger(), "Target received: x: %f, y: %f, z: %f", 
			target->x, target->y, target->z);

		//if time out
		if (!this->target_client_ptr_->wait_for_action_server()) {
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
		}

		//fetches the goal from ArmAngles.action
		auto goal_msg = SetTarget::Goal();

        goal_msg.target_angles = goal_angles;

		if (goal_msg.target_angles.size() ==0){
			RCLCPP_ERROR(this->get_logger(), "No goal angles recieved, shutting down.");
			rclcpp::shutdown();
			return;
		}

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
		};
		
		this->target_client_ptr_->async_send_goal(goal_msg, send_goal_options);
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
	//auto angles_client = std::make_shared<GetAnglesClient>(rclcpp::NodeOptions(), argc, argv);
	//std::vector<float> goal_angles = angles_client->get_angles();
	rclcpp::spin(std::make_shared<SetTargetClient>());
	rclcpp::shutdown();
	return 0;
}