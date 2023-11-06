#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Dense"


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>

#include <caric_mission/CreatePPComTopic.h>
#include "utility.h"
#include "Astar.h"
#include "grid_map.h"



class mainbrain{
public:
    mainbrain() {}
    mainbrain (string task_msg, string name)
    {
        drone_rotation_matrix = Eigen::Matrix3d::Identity();
        grid_size = Eigen::Vector3d(safe_distance, safe_distance, safe_distance);
        global_map = grid_map(grid_size);
        namespace_ = name;
        if (namespace_ == "/jurong" || namespace_ == "/raffles")
        {
            is_leader = true;
        }
        vector<string> splited_str;
        std::istringstream iss(task_msg);
        std::string substring;
        while (std::getline(iss, substring, '|'))
        {
            splited_str.push_back(substring);
        }
        if(splited_str.size() < 2){
            cout << "Bbox Assigned Error!!!" << endl;
        } else {
            if(namespace_=="/jurong"){
                vector<string> splited_splited_str;
                std::istringstream iss(splited_str[0]);
                std::string substring;
                while (std::getline(iss, substring, ';'))
                {
                    splited_splited_str.push_back(substring);
                }
                for(int i=0;i<splited_splited_str.size();i++){
                    Boundingbox bbox = Boundingbox(splited_splited_str[i]);
                    assigned_bbox_set.push_back(bbox);
                }
            }
            else{
                vector<string> splited_splited_str;
                std::istringstream iss(splited_str[1]);
                std::string substring;
                while (std::getline(iss, substring, ';'))
                {
                    splited_splited_str.push_back(substring);
                }
                for(int i=0;i<splited_splited_str.size();i++){
                    Boundingbox bbox = Boundingbox(splited_splited_str[i]);
                    assigned_bbox_set.push_back(bbox);
                }
            }
        }
        finish_init = true;
    }

    void update_map(const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &Nbr, const nav_msgs::OdometryConstPtr &msg)
    {
        if (!is_leader)
        {
            return;
        }
        Eigen::Vector3d sync_my_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        vector<Eigen::Vector3d> Nbr_point;
        Nbr_point.push_back(sync_my_position);
        CloudOdomPtr Nbr_cloud(new CloudOdom());
        pcl::fromROSMsg(*Nbr, *Nbr_cloud);
        for (const auto &point : Nbr_cloud->points)
        {
            Eigen::Vector3d cloud_point(point.x, point.y, point.z);
            if (std::fabs(point.t - Nbr->header.stamp.toSec()) > 0.2)
            {
                cout << "Missing Nbr" << endl;
            }
            else
            {
                Nbr_point.push_back(cloud_point);
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *cloud_cloud);
        for (const auto &point : cloud_cloud->points)
        {
            Eigen::Vector3d cloud_cloud_point(point.x, point.y, point.z);
            if (is_Nbr(cloud_cloud_point, Nbr_point))
            {
                continue;
            }
            else
            {
                insert_point(cloud_cloud_point);
            }
        }
    }

    void update_gimbal(Eigen::Vector3d gimbal_position)
    {
        if (!finish_init)
        {
            return;
        }
        Eigen::Matrix3d gimbal_rotation_matrix = Rpy2Rot(gimbal_position);
        Eigen::Matrix3d now_rot = gimbal_rotation_matrix * drone_rotation_matrix;
        Eigen::Vector3d rpy = Rot2rpy(now_rot);
        rpy.x() = 0;
        global_map.update_gimbal(rpy, false);

    }

    void update_position(Eigen::Vector3d point, Eigen::Matrix3d rotation)
    {
        if (!odom_get)
        {
            initial_position = point;
            initial_position += Eigen::Vector3d(0, 0, 3);
        }
        drone_rotation_matrix = rotation;
        now_global_position = point;
        if (!finish_init)
        {
            return;
        }
        global_map.update_position(point);
        odom_get = true;
    }
    
    void replan()
    {
        if (!finish_init)
        {
            return;
        }

        if (namespace_ == "/jurong" || namespace_ == "/raffles"){
            Eigen::Vector3d target = assigned_bbox_set[cur_bbox_index].getVertices()[cur_vertex_index];
            bool flag = false;
            global_map.Astar_local(target, namespace_, namespace_, flag, false);
            waypoint_get = update_target_waypoint();
            path_show = global_map.get_path_show();
            if(flag){
                if(cur_vertex_index == 7){
                    if(cur_bbox_index == (assigned_bbox_set.size() - 1)){
                        return;
                    }
                    cur_bbox_index++;
                    cur_vertex_index = 0;
                }
                cur_vertex_index++;
            }
        }
    }



    bool update_target_waypoint()
    {
        if (odom_get && finish_init)
        {
            target_position = global_map.get_next_point(true);
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_cmd(trajectory_msgs::MultiDOFJointTrajectory &cmd, geometry_msgs::Twist &gimbal)
    {
        if (!finish_first_planning)
        {
            return false;
        }
        if (waypoint_get)
        {
            global_map.get_gimbal_rpy(target_angle_rpy);
            cmd = position_msg_build(now_global_position, target_position, target_angle_rpy.z());
            gimbal = gimbal_msg_build(target_angle_rpy);
            return true;
        }
        else
        {
            return false;
        }
    }

    

private:
    Eigen::Vector3d grid_size;
    double safe_distance = 2.5;
    grid_map global_map;
    bool finish_init = false;
    Eigen::Matrix3d drone_rotation_matrix;
    bool is_leader = false;
    string namespace_;
    bool finish_first_planning = false;
    Eigen::Vector3d target_position;
    Eigen::Vector3d target_angle_rpy;
    bool waypoint_get = false;
    Eigen::Vector3d now_global_position;

    //for communication
    bool is_planned = false;
    Eigen::Vector3d planning_point;



    nav_msgs::Path path_show;
    bool odom_get = false;

    Eigen::Vector3d initial_position;

    vector<Boundingbox> assigned_bbox_set;

    int cur_bbox_index = 0;
    int cur_vertex_index = 0;


    bool is_Nbr(Eigen::Vector3d test, vector<Eigen::Vector3d> Nbr_point)
    {
        if (Nbr_point.size() == 0)
        {
            return false;
        }
        else
        {
            Eigen::Vector3d collision_box_size = grid_size;
            for (int i = 0; i < Nbr_point.size(); i++)
            {
                Eigen::Vector3d Nbr = Nbr_point[i];
                Eigen::Vector3d diff = Nbr - test;
                if (fabs(diff[0]) <= collision_box_size[0] && fabs(diff[1]) <= collision_box_size[1] && fabs(diff[2]) <= collision_box_size[2])
                {
                    return true;
                }
            }
            return false;
        }
    }

    void insert_point(Eigen::Vector3d point_in)
    {
        global_map.insert_point(point_in);
    }

    trajectory_msgs::MultiDOFJointTrajectory position_msg_build(Eigen::Vector3d position, Eigen::Vector3d target, double target_yaw)
    {
        //for communication
        is_planned = true;
        planning_point = target;


        if (fabs(target_yaw) < M_PI / 2)
        {
            target_yaw = 0;
        }
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        trajset_msg.header.frame_id = "world";
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;

        Eigen::Vector3d difference = (target - position);
        if (difference.norm() < 2)
        {
            transform_msg.translation.x = target.x();
            transform_msg.translation.y = target.y();
            transform_msg.translation.z = target.z();
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;
        }
        else
        {
            Eigen::Vector3d target_pos = 2 * difference / difference.norm();
            transform_msg.translation.x = 0;
            transform_msg.translation.y = 0;
            transform_msg.translation.z = 0;
            vel_msg.linear.x = target_pos.x();
            vel_msg.linear.y = target_pos.y();
            vel_msg.linear.z = target_pos.z();
        }
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = sinf(target_yaw * 0.5);
        transform_msg.rotation.w = cosf(target_yaw * 0.5);

        trajpt_msg.transforms.push_back(transform_msg);

        accel_msg.linear.x = 0;
        accel_msg.linear.y = 0;
        accel_msg.linear.z = 0;

        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajset_msg.header.frame_id = "world";
        return trajset_msg;
    }

    geometry_msgs::Twist gimbal_msg_build(Eigen::Vector3d target_euler_rpy)
    {
        geometry_msgs::Twist gimbal_msg;
        gimbal_msg.linear.x = 1.0; // setting linear.x to -1.0 enables velocity control mode.
        if (fabs(target_euler_rpy.z()) < M_PI / 2)
        {
            gimbal_msg.linear.y = target_euler_rpy.y(); // if linear.x set to 1.0, linear,y and linear.z are the
            gimbal_msg.linear.z = target_euler_rpy.z(); // target pitch and yaw angle, respectively.
        }
        else
        {
            gimbal_msg.linear.y = target_euler_rpy.y(); // if linear.x set to 1.0, linear,y and linear.z are the
            gimbal_msg.linear.z = 0;                    // target pitch and yaw angle, respectively.
        }
        gimbal_msg.angular.x = 0.0;
        gimbal_msg.angular.y = 0.0; // in velocity control mode, this is the target pitch velocity
        gimbal_msg.angular.z = 0.0; // in velocity control mode, this is the target yaw velocity
        return gimbal_msg;
    }
    Eigen::Matrix3d Rpy2Rot(Eigen::Vector3d rpy)
    {
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        result = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        return result;
    }
    Eigen::Vector3d Rot2rpy(Eigen::Matrix3d R)
    {

        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d rpy(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        rpy(0) = r;
        rpy(1) = p;
        rpy(2) = y;

        return rpy;
    }

};


class VixionAgent
{
public: 
    VixionAgent(ros::NodeHandlePtr &nh_ptr) : nh_ptr(nh_ptr)
    {
        //TimerProbeNbr = nh_ptr->createTimer(ros::Duration(1.0 / 10.0), &VixionAgent::TimerProbeNbrCB, this);
        TimerPlan     = nh_ptr->createTimer(ros::Duration(1.0 / 2.0),  &VixionAgent::TimerPlanCB,     this);
        TimerCmdOut   = nh_ptr->createTimer(ros::Duration(1.0 / 10.0), &VixionAgent::TimerCmdOutCB,   this);
        //TimerViz      = nh_ptr->createTimer(ros::Duration(1.0 / 1.0),  &VixionAgent::TimerVizCB,      this);

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

        cloud_sub_       = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh_ptr, "/cloud_inW", 10);
        nbr_sub_         = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh_ptr, "/nbr_odom_cloud", 10);
        odom_filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(*nh_ptr, "/ground_truth/odometry", 10);
        sync_            = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *cloud_sub_, *nbr_sub_, *odom_filter_sub_);

        sync_->registerCallback(boost::bind(&VixionAgent::MapCallback, this, _1, _2, _3));
    }

private:
    ros::NodeHandlePtr nh_ptr;
    ros::Publisher motion_pub_;
    ros::Publisher gimbal_pub_;
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
    mainbrain mm;

    ros::Timer TimerProbeNbr;   // To request updates from neighbours
    ros::Timer TimerPlan;       // To design a trajectory
    ros::Timer TimerCmdOut;     // To issue control setpoint to unicon
    ros::Timer TimerViz;        // To vizualize internal states 

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *nbr_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>       *odom_filter_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2,
                                                            nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync_;

    void OdomCallback(const nav_msgs::OdometryConstPtr &msg){
        if(!map_initialise)
        {
            Eigen::Vector3d initial_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            std_msgs::String init_position_msg;
            init_position_msg.data="init_pos;"+nh_ptr->getNamespace()+";"+to_string(initial_position.x())+","+to_string(initial_position.y())+","+to_string(initial_position.z());
            if(communication_initialise)
            {
                communication_pub_.publish(init_position_msg);
            }
            return;
        }

        Eigen::Vector3d my_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Matrix3d R = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
        mm.update_position(my_position, R);
    }

    void GimbalCallback(const geometry_msgs::TwistStamped &msg)
    {
        if(!map_initialise)
        {
            return;
        }
        Eigen::Vector3d position = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
        mm.update_gimbal(position);
    }
    

    void TimerPlanCB(const ros::TimerEvent &)
    {
        if (!map_initialise)
        {
            return;
        }
       ////yolo();
        mm.replan();
       ////yolo();
        return;
    }


    void TimerCmdOutCB(const ros::TimerEvent &){
        if(!map_initialise) return;
        
        trajectory_msgs::MultiDOFJointTrajectory position_cmd;
        geometry_msgs::Twist gimbal_msg;
        if (mm.get_cmd(position_cmd, gimbal_msg))
        {
            position_cmd.header.stamp = ros::Time::now();
            motion_pub_.publish(position_cmd);
            gimbal_pub_.publish(gimbal_msg);
        }

        return;
    }
    
    void TaskCallback(const std_msgs::String &msg){
        map_initialise = true;
        mm = mainbrain(msg.data, nh_ptr->getNamespace());
    }
    
    void ComCallback(){
    
    } 

    void MapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                     const sensor_msgs::PointCloud2ConstPtr &Nbr,
                     const nav_msgs::OdometryConstPtr &msg)
    {
        if(!map_initialise) return;

        if(std::fabs(cloud->header.stamp.toSec() - Nbr->header.stamp.toSec())>0.2) return;

        mm.update_map(cloud, Nbr, msg);
    }
};