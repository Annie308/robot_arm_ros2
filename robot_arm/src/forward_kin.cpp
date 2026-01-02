#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <future>

#include "geometry_msgs/msg/vector3.hpp"

#include "rclcpp/rclcpp.hpp"
#include "robot_arm_interfaces/msg/joint_positions.hpp"
#include "robot_arm_interfaces/msg/joint_angles.hpp"
#include <Eigen/Dense>

#include "arm_attributes.h"

using namespace std::chrono_literals;
using namespace std::placeholders;
using JointPositions = robot_arm_interfaces::msg::JointPositions;
using JointAngles = robot_arm_interfaces::msg::JointAngles;	

const int JOINTS_NUM = 6;

//Forward kinematics: joint angles to cartesian
std::vector<geometry_msgs::msg::Vector3> fk(float t1, float t2, float t3, float t4, float t5, float t6) {

	//Rotation transformation matrices
	// Rotation about Z
	Eigen::MatrixXd rot1(3, 3);
	rot1 << cos(t1), -sin(t1), 0,
		sin(t1), cos(t1), 0,
		0, 0, 1;

	//Y
	Eigen::MatrixXd rot2(3, 3);
	rot2 << cos(t2), 0, sin(t2),
		0, 1, 0,
		-sin(t2), 0, cos(t2);
	//Y
	Eigen::MatrixXd rot3(3, 3);
	rot3 << cos(t3), 0, sin(t3),
		0, 1, 0,
		-sin(t3), 0, cos(t3);

	//roll about X
	Eigen::MatrixXd rot4(3, 3);
	rot4 << 1, 0, 0,
		0, cos(t4), -sin(t4),
		0, sin(t4), cos(t4);

	//roll about Y
	Eigen::MatrixXd  rot6(3, 3);
	rot6 << cos(t6), -sin(t6), 0,
		sin(t6), cos(t6), 0,
		0, 0, 1;

	//pitch about Z
	Eigen::MatrixXd rot5(3, 3);
	rot5 << cos(t5), 0, sin(t5),
		0, 1, 0,
		-sin(t5), 0, cos(t5);

	

	//defining joint positons after transformations
	Eigen::Vector3d link1_end = rot1 * link1;
	Eigen::Vector3d link2_end = rot1 * rot2 * link2 + link1_end;
	Eigen::Vector3d link3_end = rot1 * rot2 * rot3 * link3 + link2_end;
	Eigen::Vector3d link4_end = rot4 * link4 + link3_end;
	Eigen::Vector3d link5_end = rot4 * rot5 * link5 + link4_end;
	Eigen::Vector3d link6_end = rot4 * rot5 * rot6 * link6 + link5_end;

	std::vector<Eigen::Vector3d> joint_positions_eigen { link1_end, link2_end, link3_end, link4_end, link5_end,link6_end};
	std::vector<geometry_msgs::msg::Vector3> joint_positions;

	for (const auto& pos : joint_positions_eigen) {
		geometry_msgs::msg::Vector3 vec;
		vec.x = pos(0);
		vec.y = pos(1);
		vec.z = pos(2);
		joint_positions.push_back(vec);
	}

	return joint_positions;
}


class ForwardKinNode : public rclcpp::Node
{
public:

	ForwardKinNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		: Node("forward_kin_node", options)
	{
		publisher_ = this->create_publisher<JointPositions>(
			"joint_positions", 10);

		subscription_ = this->create_subscription<robot_arm_interfaces::msg::JointAngles>(
			"gyro_arm_angles", 10, std::bind(&ForwardKinNode::topic_callback, this, _1));

		auto subscriber_timer_callback =
			[this](){return std::bind(&ForwardKinNode::topic_callback, this, _1);};
			subscription_timer_ = this->create_wall_timer(2000ms, subscriber_timer_callback);
			

		 auto publisher_timer_callback =
			[this]() -> void {
				auto message = JointPositions();

				message.joint_positions = curr_positions;

				if (curr_positions.size() !=0){
					RCLCPP_INFO(this->get_logger(), "Publishing joint states: ");
					for (size_t i=0; i < message.joint_positions.size(); i++){
						RCLCPP_INFO(this->get_logger(), "Joint %zu: x: %f, y: %f, z: %f", i+1,
							message.joint_positions[i].x,
							message.joint_positions[i].y,
							message.joint_positions[i].z);
					}
					this->publisher_->publish(message);
				}
			};
			publisher_timer_ = this->create_wall_timer(2000ms, publisher_timer_callback);
	}

private:
	void topic_callback(const JointAngles::SharedPtr msg){
		for (size_t i=0; i<msg->joint_angles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg->joint_angles[i]);
		}
		
		if (msg->joint_angles.size() != JOINTS_NUM){
			RCLCPP_ERROR(this->get_logger(), "Invalid number of joint angles received");
			return;
		}
		curr_positions = fk(
			static_cast<float>(msg->joint_angles[0]),
			static_cast<float>(msg->joint_angles[1]),
			static_cast<float>(msg->joint_angles[2]),
			static_cast<float>(msg->joint_angles[3]),
			static_cast<float>(msg->joint_angles[4]),
			static_cast<float>(msg->joint_angles[5])
		);
	}
	rclcpp::Publisher<JointPositions>::SharedPtr publisher_;
	rclcpp::Subscription<JointAngles>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr publisher_timer_;
	rclcpp::TimerBase::SharedPtr subscription_timer_;

	std::vector<geometry_msgs::msg::Vector3> curr_positions;
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForwardKinNode>());
	rclcpp::shutdown();
	return 0;
}