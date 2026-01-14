#include <memory>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <sstream>
#include <iostream>
#include <optional>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_arm_interfaces/msg/joint_positions.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include "robot_arm_interfaces/srv/set_claw.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

using JointPositions = robot_arm_interfaces::msg::JointPositions;
using InverseKin = robot_arm_interfaces::srv::InverseKin;
using SetClaw = robot_arm_interfaces::srv::SetClaw;

enum class GoalType {RELEASE_CLAW, CLAMP_CLAW, CONFIG_ARM};

class ServiceClient : public rclcpp::Node{
public:
	 ServiceClient(const rclcpp::NodeOptions & options, geometry_msgs::msg::Transform target_, GoalType goal_type_) 
    : Node("service_client", options),  target(target_), goal_type(goal_type_)
  {	
		angles_client_ptr_ =this->create_client<InverseKin>("target_angles");
		claw_client_ptr_ =this->create_client<SetClaw>("set_claw");	
	}

	void get_angles(){

		auto request = std::make_shared<InverseKin::Request>();
		request->target.translation.x = target.translation.x;
		request->target.translation.y = target.translation.y;
		request->target.translation.z = target.translation.z;

		request->target.rotation.x = target.rotation.x;
		request->target.rotation.y = target.rotation.y;
		request->target.rotation.z = target.rotation.z;

		RCLCPP_INFO(this->get_logger(), "Sending inverse kin service request: %lf, %lf, %lf, %lf, %lf, %lf", 
		request->target.translation.x,  request->target.translation.y, request->target.translation.z,
		request->target.rotation.x, request->target.rotation.y, request->target.rotation.z);
		
		
        while (!angles_client_ptr_->wait_for_service(5s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
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
				RCLCPP_INFO(this->get_logger(), "%lf ", angle);
			}
			return;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_angles");
        }
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
		geometry_msgs::msg::Transform target;
		GoalType goal_type;
};

//sends the target goal to the motors and is subsrcibed to the gyro sensors to get the current angles
class SetTargetClient : public rclcpp::Node{
public:
	
  SetTargetClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  	: Node("set_target_client", options)
  {	
	target = get_user_input();

	if (goal_type == GoalType::CONFIG_ARM){
		auto angles_client = std::make_shared<ServiceClient>(rclcpp::NodeOptions(), *target, goal_type);
		angles_client->get_angles();
	}else{

		//create client
		auto claw_client = std::make_shared<ServiceClient>(rclcpp::NodeOptions(), *target, goal_type);
		claw_client->send_claw_request();
	}
	
	}
private:

	rclcpp::TimerBase::SharedPtr timer_;

	std::optional<std::vector<double>> goal_angles;
	std::optional<geometry_msgs::msg::Transform> target;

	GoalType goal_type;
	int goal_type_param;


	std::optional<geometry_msgs::msg::Transform> get_user_input()
	{	
		std::string set_goal_type;
		geometry_msgs::msg::Transform target_;

		RCLCPP_INFO(this->get_logger(),
			"Enter separated by spaces:\n"
			"(1) Configure arm\n"
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
				RCLCPP_INFO(this->get_logger(), "Request received: Configure arm");
				goal_type = GoalType::CONFIG_ARM;
				break;
			case '2':
				RCLCPP_INFO(this->get_logger(), "Request received: Clamp claw");
				goal_type = GoalType::CLAMP_CLAW;
				break;
			case '3':
				RCLCPP_INFO(this->get_logger(), "Request received: Release claw");
				goal_type = GoalType::RELEASE_CLAW;
				break;
			default:
				RCLCPP_ERROR(this->get_logger(),"Request received: Invalid input");
				return std::nullopt;
		}

		if (goal_type== GoalType::CONFIG_ARM){
			RCLCPP_INFO(this->get_logger(), "Enter target arm position as x y z >> ");

				std::string str_target;
				std::getline(std::cin, str_target);  // read the whole line, including spaces
				std::stringstream ss(str_target);     // put it into a stringstream

				double x, y, z;

				if (!(ss >> x >> y >> z)){
					RCLCPP_ERROR(this->get_logger(), "Invalid arm target input");
					return std::nullopt;
				}

				target_.translation.x = x;
				target_.translation.y = y;
				target_.translation.z = z;

			RCLCPP_INFO(this->get_logger(), "Enter target wrist rotations as roll, pitch, yaw (in radians) >> ");

				std::string str_rotation;
				std::getline(std::cin, str_rotation);  // read the whole line, including spaces
				std::stringstream ss_(str_rotation);     // put it into a stringstream

				double roll, pitch, yaw;

				if (!(ss_ >> roll >>pitch >> yaw)){
					RCLCPP_ERROR(this->get_logger(), "Invalid wrist target input");
					return std::nullopt;
				}
				//i know this is supposed to be a quarternion but idk how to use it yet
				target_.rotation.x = roll;
				target_.rotation.y = pitch;
				target_.rotation.z = yaw;
		}

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