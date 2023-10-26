#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Dense"
#include <caric_mission/CreatePPComTopic.h>


caric_mission::CreatePPComTopic srv;
ros::ServiceClient client;
ros::Publisher cmd_pub_;
bool serviceAvailable = false;

int main(int argc, char **argv){
    ros::init(argc, argv, "gcs_controller");
    ros::NodeHandle nh;
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    client = nh_ptr->serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
    cmd_pub_ = nh_ptr->advertise<std_msgs::String>("/task_assign", 10);
    srv.request.source = "gcs";
    srv.request.targets.push_back("all");
    srv.request.topic_name = "/task_assign";
    srv.request.package_name = "std_msgs";
    srv.request.message_type = "String";
    while (!serviceAvailable)
    {
        serviceAvailable = ros::service::waitForService("create_ppcom_topic", ros::Duration(10.0));
    }
    client.call(srv);
}