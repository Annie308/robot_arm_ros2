#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#include <cmath>
#include <thread>
#include <chrono>


using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options)
        { 
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
            // create a publisher to tell robot_state_publisher the JointState information.
            // robot_state_publisher will deal with this transformation
            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            // create a broadcaster to tell the tf2 state information
            // this broadcaster will determine the position of coordinate system 'axis' in coordinate system 'odom'
            RCLCPP_INFO(this->get_logger(),"Starting state publisher");

            loop_rate_=std::make_shared<rclcpp::Rate>(100ms);

            publisher_timer_=this->create_wall_timer(100ms,std::bind(&StatePublisher::publish,this));

            subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "arm_angles", 10, std::bind(&StatePublisher::topic_callback, this, std::placeholders::_1));

            auto subscriber_timer_callback =
            [this](){return std::bind(&StatePublisher::topic_callback, this,  std::placeholders::_1);};
            subscription_timer_ = this->create_wall_timer(500ms, subscriber_timer_callback);

            angles_vec.resize(6, 0.);
            angles_rec.resize(6, 0.);
        }

        void publish();

        void topic_callback(const std_msgs::msg::Float64MultiArray msg){
            for (size_t i=0; i<msg.data.size(); i++){
                RCLCPP_INFO(this->get_logger(), "Angles Recieved: '%lf'", msg.data[i]);
                if (i < angles_rec.size()) { angles_rec[i] = msg.data[i]; }
            }
         }
    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr publisher_timer_;
	rclcpp::TimerBase::SharedPtr subscription_timer_;

    std::vector<double> angles_rec;
    const double degree=M_PI/180.0;
    std::vector<double> angles_vec;
    double r1, r2, r3, r4, r5, r6; 
    
};

void StatePublisher::publish(){
     //Robot state variables
    // degree means one degree

    // create the necessary messages
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    // add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"base_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_link4", "link4_to_link5_pitch","link4_to_link5_roll" };
    joint_state.position={r1,r2, r3, r4, r5, r6};

    // add time stamp
    t.header.stamp=this->get_clock()->now();
    // specify the father and child frame

    // odom is the base coordinate system of tf2
    t.header.frame_id="odom";
    // axis is defined in r2d2.urdf.xml file and it is the base coordinate of model
    t.child_frame_id="axis";

    double tol = 0.9*degree;

    for (size_t i=0; i< angles_vec.size(); i++){
        if (std::fabs(angles_vec[i] - angles_rec[i]) > tol){
            if (angles_rec[i]< 0){
                angles_vec[i] -= degree;
            }else if (angles_rec[i] > 0){
                angles_vec[i] += degree;
            }
        }
    }

   bool all_similar = true;

    for (size_t i = 0; i < angles_rec.size(); i++) {
        if (std::fabs(angles_vec[i] - angles_rec[i]) > tol) {
            all_similar = false;
            break;
        }
    }

    if (all_similar) {
       // std::fill(angles_vec.begin(), angles_vec.end(), 0.0);
    }

    r1 = angles_vec[0];
    r2 = angles_vec[1];
    r3 = angles_vec[2];
    r4 = angles_vec[3];
    r5 = angles_vec[4];
    r6 = angles_vec[5];

    // send message
    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    for (size_t i =0; i< angles_vec.size(); i++){
        RCLCPP_INFO(this->get_logger(),"Publishing angle %zu: %lf",i, angles_vec[i]);
    }
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}