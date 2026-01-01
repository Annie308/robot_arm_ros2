#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_arm_interfaces/msg/joint_angles.hpp"

#include <cmath>
#include <thread>
#include <chrono>

using JointAngles = robot_arm_interfaces::msg::JointAngles;

using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
            // create a publisher to tell robot_state_publisher the JointState information.
            // robot_state_publisher will deal with this transformation
            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            // create a broadcaster to tell the tf2 state information
            // this broadcaster will determine the position of coordinate system 'axis' in coordinate system 'odom'
            RCLCPP_INFO(this->get_logger(),"Starting state publisher");

            loop_rate_=std::make_shared<rclcpp::Rate>(100ms);

            publisher_timer_=this->create_wall_timer(100ms,std::bind(&StatePublisher::publish,this));

            subscription_ = this->create_subscription<robot_arm_interfaces::msg::JointAngles>(
			"gyro_arm_angles", 10, std::bind(&StatePublisher::topic_callback, this, std::placeholders::_1));

		    auto subscriber_timer_callback =
			[this](){return std::bind(&StatePublisher::topic_callback, this,  std::placeholders::_1);};
			subscription_timer_ = this->create_wall_timer(2000ms, subscriber_timer_callback);
        }

    void publish();

    void topic_callback(const JointAngles::SharedPtr msg){
		for (size_t i=0; i<msg->joint_angles.size(); i++){
			RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg->joint_angles[i]);
		}
    }

    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate_;

    rclcpp::Subscription<JointAngles>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr publisher_timer_;
	rclcpp::TimerBase::SharedPtr subscription_timer_;

    //Robot state variables
    // degree means one degree
    const double degree=M_PI/180.0;
    double r1 = 0.;
    double r2 = 0.;
    double r3 = 0.;
    double r4 = 0.;
    double r5 = 0.;
    double r6 = 0.;
    double angle_r1 = 0.;
    double angle_r2 = 0.;
};

void StatePublisher::publish(){
    // create the necessary messages
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    // add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"base_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_link4", "link4_to_link5", "connector2_to_link6"};
    joint_state.position={r1,r2,r3, r4, r5, r6};

    // add time stamp
    t.header.stamp=this->get_clock()->now();
    // specify the father and child frame

    // odom is the base coordinate system of tf2
    t.header.frame_id="odom";
    // axis is defined in r2d2.urdf.xml file and it is the base coordinate of model
    t.child_frame_id="axis";

    /*
    angle_r1+=degree/5;
    if (angle_r1<-3.14 || angle_r1>3.14){
        angle_r1*=-1;
    }

    angle_r2 +=degree/5;
    if (angle_r2<-1.57 || angle_r2>1.57){
        angle_r2*=-1;
    }
        */
        
    
    r1+=angle_r1;  // Increment by 1 degree (in radians)
    r2+= angle_r2;  // Increment by 0.5 degree (in radians)
    r3 += angle_r2;

    // send message
    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    RCLCPP_INFO(this->get_logger(),"Publishing joint state");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}