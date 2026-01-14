#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "robot_arm_interfaces/srv/inverse_kin.hpp"

#include <Eigen/Dense>

#include "arm_attributes.h"

using namespace std::chrono_literals;
using namespace std::placeholders;
using InverseKin = robot_arm_interfaces::srv::InverseKin;

static std::tuple<double,double,double> wrist_ik(double t1, double t2, double t3,double roll, double pitch, double yaw) {
	//First we find R3 from the arm
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

	Eigen::MatrixXd R_13 = rot1*rot2*rot3;

	//getting desired rotation matrices
	//X
	//X
	Eigen::MatrixXd R_roll(3, 3);
	R_roll << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);
	
	//Z
	Eigen::MatrixXd R_pitch(3, 3);
	R_pitch << cos(pitch), 0, sin(pitch),
		0, 1, 0,
		-sin(pitch), 0, cos(pitch);
	
	//Y
	Eigen::MatrixXd R_yaw(3, 3);
	R_yaw << cos(yaw), -sin(yaw), 0,
		sin(yaw), cos(yaw), 0,
		0, 0, 1;

	//Goal rotations
	Eigen::MatrixXd R_T =R_roll*R_pitch*R_yaw;

	Eigen::MatrixXd goalRot = R_13.transpose() * R_T; // desired relative wrist rotation

	//constraints
	//-pi <= t4 <= pi
	//-pi/2 <= t5 <= pi/2
	//-pi <= t6 <= pi
	
	double t4, t5, t6;

	//Our wrist rotation is in XYZ. Im just gonna take only one solutions (the positive one) for now

	//constraints
	double cos_t5 = std::clamp(sqrt(goalRot(0, 0)*goalRot(0,0) + goalRot(0,1)*goalRot(0,1)), -1.0, 1.0);
	double sin_t5 = std::clamp(goalRot(0,2), -1.0, 1.0);

	t5 = atan2(sin_t5,cos_t5);			//i think there may be only 1 solution since cos is an even function
	
	//check for gimbal lock
	bool gimbalLock = fabs(fabs(sin_t5) - 1.0) < 1e-5;

	if (gimbalLock) {
		// Snap to 90
		t5 = (sin_t5 > 0 ? PI/2 : -PI/2);
		t4 = 0.0;
		t6 = atan2(goalRot(0, 1), goalRot(0, 2));
	}
	else {
		t4 = atan2(-goalRot(1, 2)/cos(t5), goalRot(2, 2)/cos(t5));
		t6 = atan2(-goalRot(0, 1)/cos(t5), goalRot(0, 0)/cos(t5));
	}
	return std::make_tuple(t4, t5, t6);
}

static std::tuple<double,double,double> ik(double x, double y, double z) {
	Eigen::Vector3d p_target = Eigen::Vector3d(x, y, z);

	double l3_eff = l3 + l4;
	
	p_target = p_target -link1 - base_link;

	//angle away from XY plane
	double t = atan2(p_target.y(), p_target.x());

	//rotate on the z axis to project onto XZ plane
	Eigen::MatrixXd proj_max(3, 3);
	proj_max << cos(-t), -sin(-t), 0,
		sin(-t), cos(-t), 0,
		0, 0, 1;

	//projected point 
	Eigen::Vector3d p_proj = proj_max * p_target;

	//defining new coordinates 
	double x1 = p_proj.x(), z1 = p_proj.z();

	double r1 = sqrt(x1 * x1 + z1 * z1);			//new length
	double phi = atan2(z1, x1);					//angle from base to projected point

	//angle from the new point using cosine law
	double cos_t2 = (l2 * l2 + r1 * r1 - (l3_eff) * (l3_eff)) / (2. * l2 * r1);
	cos_t2 = std::min(1.0, std::max(-1.0, cos_t2));						//constraints

	double cos_t3 = (l2 * l2 + (l3_eff) * (l3_eff)-r1 * r1) / (2. * l2 * (l3_eff));
	cos_t3 = std::min(1.0, std::max(-1.0, cos_t3));						//constraints			

	double t2 = -phi + acos(cos_t2);
	double t3 = -PI + acos(cos_t3);
	double t1 = t;

	return std::make_tuple(t1, t2, t3);
}

class IkServer : public rclcpp::Node
{
public:
	
  IkServer(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  	: Node("ik_server", options)
  {	
	//a client needs 3 things:
	/*
	1.The templated action type name: GetArmAngles.
	2.A ROS 2 node to add the action client to: this.
	3. The action name: 'arm_angles'.
	*/
    service_ptr_ = this->create_service<InverseKin>("target_angles", 
		std::bind(&IkServer::get_result, this, _1, _2)
	);

	publisher_ptr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_angles", 10);

	auto publisher_timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::Float64MultiArray();

        message.data = joint_angles;

		RCLCPP_INFO(this->get_logger(), "Publishing:");
		
		this->publisher_ptr_->publish(message);
		
		for (auto angle: joint_angles){
        	RCLCPP_INFO(this->get_logger(), "%lf", angle);
		}
       
      };

    publisher_timer_ = this->create_wall_timer(2000ms, publisher_timer_callback);
  	RCLCPP_INFO(this->get_logger(), "Ready to calculate target joint angles.");
        
    }
	
private:
  rclcpp::Service<InverseKin>::SharedPtr service_ptr_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_ptr_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  std::vector<double> joint_angles;

  enum class GoalType {SET_ARM_POS, SET_WRIST_ROTATION};

  void get_result(const std::shared_ptr<InverseKin::Request> request,std::shared_ptr<InverseKin::Response>response)
  {
		double x = request->target.translation.x; 
		double y = request->target.translation.y; 
		double z =request->target.translation.z;

		double roll = request->target.rotation.x;
		double pitch = request->target.rotation.y;
		double yaw = request->target.rotation.z;

				
		std::tuple<double,double, double> arm_result = ik(x, y, z);

		double t1 = std::get<0>(arm_result);
		double t2 = std::get<1>(arm_result);
		double t3 = std::get<2>(arm_result);

		RCLCPP_INFO(this->get_logger(), "Incoming request for arm angles\n");
		response->angles.push_back(t1);
		response->angles.push_back(t2);
		response->angles.push_back(t3);

		std::tuple<double,double,double> wrist_result = wrist_ik(t1, t2, t3, roll, pitch, yaw);
		RCLCPP_INFO(this->get_logger(), "Incoming request for wrist angles\n");
		response->angles.push_back(std::get<0>(wrist_result));
		response->angles.push_back(std::get<1>(wrist_result));
		response->angles.push_back(std::get<2>(wrist_result));
		
		
		RCLCPP_INFO(this->get_logger(), "x: %lf\ny: %lf\nz: %lf",
			request->target.translation.x,request->target.translation.y,request->target.translation.z);
		RCLCPP_INFO(this->get_logger(), "roll: %lf\nyaw: %lf",
			request->target.rotation.y,request->target.rotation.z);
		RCLCPP_INFO(this->get_logger(), "Sending back %zu angles: ", response->angles.size());

		for (size_t i=0; i< response->angles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Sending back %zu joint angles: %lf rads...", i, response->angles[i]);
			joint_angles.push_back(response->angles[i]);
		}
  }	
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IkServer>());
	rclcpp::shutdown();
	return 0;
}