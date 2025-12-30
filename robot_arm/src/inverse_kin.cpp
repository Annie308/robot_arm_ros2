#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace std::placeholders;
using InverseKin = robot_arm_interfaces::srv::InverseKin;

//Make these parameters!!!!
//----------------------
const float l1 = 5.0f; 
const float l2 = 10.0f;
const float l3 = 10.f;
const float l4 = 3.f;
const float l5 = 3.f;
const float l6 = 3.f;
//---------------------

const float PI = 3.14159265358979323846f;

Eigen::Vector3d link1(0, l1, 0);
Eigen::Vector3d link2(l2, 0, 0);
Eigen::Vector3d link3(l3, 0, 0);

Eigen::Vector3d link4 = { l4, 0,0, };
Eigen::Vector3d link5(0, l5, 0);
Eigen::Vector3d link5_1(l5, 0, 0);
Eigen::Vector3d link6(l6, 0, 0);


static std::tuple<float,float,float> ik(float x, float y, float z) {
	Eigen::Vector3d p_target = Eigen::Vector3d(x, y, z);

	p_target = p_target - link1 - link4 - link5_1 - link6;		

	//angle away from XY plane
	float t = atan2(p_target.z(), p_target.x());

	//rotate on the y axis to project onto XY plane
	Eigen::MatrixXd proj_max(3, 3);
	proj_max << cos(t), 0, sin(t),
		0, 1, 0,
		-sin(t), 0, cos(t);

	//projected point 
	Eigen::Vector3d p_proj = proj_max * p_target;
	
	//defining new coordinates 
	float x1 = p_proj(0);
	float y1 = p_proj(1);

	float r1 = sqrt(x1*x1 + y1 * y1);			//new length
	float phi = atan2(y1, x1);					//angle from base to projected point
	
	//angle from the new point using cosine law
	float cos_t2 = (l2 * l2 + r1 * r1 - (l3) * (l3)) / (2.f * l2 * r1);
	cos_t2 = std::min(1.0f, std::max(-1.0f, cos_t2));						//constraints
	
	float cos_t3 = (l2 * l2 + (l3) * (l3) - r1 * r1) / (2.f * l2 * (l3));
	cos_t3 = std::min(1.0f, std::max(-1.0f, cos_t3));						//constraints

	//case 1: elbow up
	float t2 = phi + acos(cos_t2);
	float t3 = -PI + acos(cos_t3);
	float t1 = -t;

	/*case 2: elbow down
	if (!isReachable(t1, t2, t3)) {
		t2 = phi - acos(cos_t2);
		t3 = PI - acos(cos_t3);
	};
	Eigen::Vector3d tar_vec = fk(t1, t2, t3, 0, 0, 0).back();
  */

	return std::make_tuple(t1, t2, t3);
}

static std::tuple<float,float,float> wrist_ik(float roll, float pitch, float yaw) {

	//getting desired rotation matrices
	//roll about X
	Eigen::MatrixXd R_roll(3, 3);
	R_roll << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);

	//Y
	Eigen::MatrixXd R_yaw(3, 3);
	R_yaw << cos(yaw), 0, sin(yaw),
		0, 1, 0,
		-sin(yaw), 0, cos(yaw);

	//pitch about Z
	Eigen::MatrixXd R_pitch(3, 3);
	R_pitch << cos(pitch), -sin(pitch), 0,
		sin(pitch), cos(pitch), 0,
		0, 0, 1;

	Eigen::MatrixXd goalRot = R_roll * R_pitch * R_yaw; // relative wrist rotation
	float t4, t5, t6;

	//constraints
	float sin_t5 = std::clamp(goalRot(0, 2), -1.0, 1.0);
	t5 = asinf(sin_t5);

	//check for gimbal lock
	bool gimbalLock = fabsf(fabs(sin_t5) - 1.0f) < 1e-5f;

	if (gimbalLock) {
		// Snap to 90
		t5 = (sin_t5 > 0 ? PI/2 : -PI/2);
		t4 = 0.0f;
		t6 = atan2f(goalRot(1, 0), goalRot(1, 1));
	}
	else {
		t4 = atan2f(-goalRot(1, 2), goalRot(2, 2));
		t6 = atan2f(-goalRot(0, 1), goalRot(0, 0));
	}
	return std::make_tuple(t4, t5, t6);
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
    service_ptr_ = this->create_service<InverseKin>("target_arm_angles", 
		std::bind(&IkServer::get_result, this, _1, _2)
	);

  	RCLCPP_INFO(this->get_logger(), "Ready to calculate target joint angles.");
        
    }
	
private:
  rclcpp::Service<InverseKin>::SharedPtr service_ptr_;

  enum class GoalType {SET_ARM_POS, SET_WRIST_ROTATION};

  void get_result(const std::shared_ptr<InverseKin::Request> request,std::shared_ptr<InverseKin::Response>response)
  {
		float x = request->target.x; 
		float y = request->target.y; 
		float z =request->target.z;

		std::tuple<float,float,float> result;

		if (request->mode == 0){			//arm ik
			result = ik(x, y, z);
			RCLCPP_INFO(this->get_logger(), "Incoming request for arm angles\n");
			
		}else if (request->mode ==1){		//wrist ik
			result = wrist_ik(x, y, z);
			RCLCPP_INFO(this->get_logger(), "Incoming request for wrist angles\n");
		}else{
			RCLCPP_ERROR(this->get_logger(), "Invalid mode selected");
			return;
		}
		response->angles.push_back(std::get<0>(result));
		response->angles.push_back(std::get<1>(result));
		response->angles.push_back(std::get<2>(result));
		
		RCLCPP_INFO(this->get_logger(), "x: %f\ny: %f\nz: %f",request->target.x,request->target.y,request->target.z);
		RCLCPP_INFO(this->get_logger(), "Sending back %zu angles: ", response->angles.size());

		for (size_t i=0; i< response->angles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Theta %zu: %f rads...", i, response->angles[i]);
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