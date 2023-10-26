#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Dense"
#include <caric_mission/CreatePPComTopic.h>
#include "utility.h"

class VixionAgent
{
public: 
    VixionAgent(ros::NodeHandlePtr &nh_ptr) : nh_ptr(nh_ptr){
        TimerCmdOut = nh_ptr->createTimer(ros::Duration(1.0 / 10.0), &VixionAgent::TimerCmdOutCB, this);


        task_sub_ = nh_ptr->subscribe("/task_assign" + nh_ptr->getNamespace(), 10, &VixionAgent::TaskCallback, this);
        //com_sub_  = nh_ptr->subscribe("/broadcast" + nh_ptr->getNamespace(), 10, &ComCallback);
        client    = nh_ptr->serviceClient<caric_mission::CreatePPComTopic>("/create_ppcom_topic");
        communication_pub_ = nh_ptr->advertise<std_msgs::String>("/broadcast", 10);

        string str = nh_ptr->getNamespace();
        str.erase(0, 1);
        srv.request.source = str;
        srv.request.targets.push_back("all");
        srv.request.topic_name = "/broadcast";
        srv.request.package_name = "std_msgs";
        srv.request.message_type = "String";
        while (!serviceAvailable)
        {
            serviceAvailable = ros::service::waitForService("/create_ppcom_topic", ros::Duration(10.0));
        }
        string result = "Begin";
        while (result != "success lah!")
        {
            client.call(srv);
            result = srv.response.result;
            printf(KYEL "%s\n" RESET, result.c_str());
            std::this_thread::sleep_for(chrono::milliseconds(1000));
        }
        communication_initialise = true;

        motion_pub_ = nh_ptr->advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1000);
        odom_sub_ = nh_ptr->subscribe("/firefly/ground_truth/odometry", 10, &VixionAgent::OdomCallback, this);
    }

private:
    ros::NodeHandlePtr nh_ptr;
    ros::Publisher motion_pub_;
    ros::Subscriber odom_sub_;
    Eigen::Vector3d nextPose;
    ros::Subscriber task_sub_;
    ros::Subscriber com_sub_;
    caric_mission::CreatePPComTopic srv; // This PPcom create for communication between neibors;
    ros::ServiceClient client;           // The client to create ppcom
    ros::Publisher communication_pub_; 
    bool serviceAvailable = false;
    bool communication_initialise = false;
    bool map_initialise = false;
    ros::Timer TimerCmdOut;  



    void OdomCallback(const nav_msgs::OdometryConstPtr &msg){
        nextPose = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
    
    void TimerCmdOutCB(const ros::TimerEvent &){
        
        
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
    
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;
    
        transform_msg.translation.x = 0;
        transform_msg.translation.y = 0;
        transform_msg.translation.z = 0;
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = 0;
        transform_msg.rotation.w = 0;
    
        trajpt_msg.transforms.push_back(transform_msg);
    
        vel_msg.linear.x = -2.0 + 4.0 * (rand() % 1000) / 1000.0;
        vel_msg.linear.y = -2.0 + 4.0 * (rand() % 1000) / 1000.0;
        vel_msg.linear.z = -2.0 + 4.0 * (rand() % 1000) / 1000.0;
    
        accel_msg.linear.x = 0;
        accel_msg.linear.y = 0;
        accel_msg.linear.z = 0;
    
        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);
    
        trajset_msg.header.stamp = ros::Time::now();
        motion_pub_.publish(trajset_msg); 
    }
    
    void TaskCallback(const std_msgs::String &msg){
        map_initialise = true;
    }
    
    void ComCallback(){
    
    }
};