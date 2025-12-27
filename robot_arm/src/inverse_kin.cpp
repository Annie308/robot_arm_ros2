#include <chrono>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_arm_interfaces/srv/inverse_kin.hpp"

//Make thise parameters!!!!
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

void ik_res (const std::shared_ptr<robot_arm_interfaces::srv::InverseKin::Request> request,
          std::shared_ptr<robot_arm_interfaces::srv::InverseKin::Response>      response)
{
  float x = request->target.x; 
  float y = request->target.y; 
  float z =request->target.z;

  std::tuple<float,float,float> result = ik(x, y, z);
  response->angles.push_back(std::get<0>(result));
  response->angles.push_back(std::get<1>(result));
  response->angles.push_back(std::get<2>(result));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f\ny: %f\nz: %f",request->target.x,request->target.y,request->target.z);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back %zu angles: ", response->angles.size());

  for (size_t i=0; i< response->angles.size(); i++){
     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Theta %zu: %lf rads...", i, response->angles[i]);
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("inverse_kin");

  rclcpp::Service<robot_arm_interfaces::srv::InverseKin>::SharedPtr service =
    node->create_service<robot_arm_interfaces::srv::InverseKin>("arm_angles", &ik_res);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to find arm joint angles.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}