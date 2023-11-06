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
#include "Astar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caric_mission/CreatePPComTopic.h>

struct position_info{
    Eigen::Vector3d position;
    bool update=false;
};

class Boundingbox
{
public:
    Boundingbox(string str)
    {
        vector<string> spilited_str;
        std::istringstream iss(str);
        std::string substring;
        while (std::getline(iss, substring, ','))
        {
            spilited_str.push_back(substring);
        }
        
        int i = 0;
        while (i < 24)
        {
            vertices.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        while (i < 33)
        {
            rotation_matrix(i - 24) = stod(spilited_str[i]);
            i++;
        }
        while (i < 36)
        {
            center = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 39)
        {
            size_vector = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        //while (i < 42)
        //{
        //    global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
        //    i = i + 3;
        //}
        //while (i < 45)
        //{
        //    global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
        //    i = i + 3;
        //}
        //
        xsize = stod(spilited_str[i]);
        i++;
        ysize = stod(spilited_str[i]);
        i++;
        zsize = stod(spilited_str[i]);
        i++;
        id = stod(spilited_str[i]);
        i++;
        state = stod(spilited_str[i]);
        i++;
        volume = stod(spilited_str[i]);
        i++;
        use_x = stod(spilited_str[i]);
        i++;
        use_y = stod(spilited_str[i]);
        i++;
        use_z = stod(spilited_str[i]);
        i++;
    }
    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in)
    {
        vertices = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;
        xsize = (vec[1] - vec[0]).norm();
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();

        volume = xsize * ysize * zsize;
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);

        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();

        rotation_matrix << xaxis, yaxis, zaxis;
    };

    ~Boundingbox(){};
    
    const Matrix3d getSearchRotation() const
    {
        Eigen::Vector3d axis_rotation_along(0.0, 0.0, 1.0);
        Eigen::Matrix3d transfer_matrix;
        Eigen::Matrix3d result;
        double angle = 0;
        if (use_x)
        {
            if (state == 0)
            {
                cout << "x+" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = -M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "x-" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                cout << "y+" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "y-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = -M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                cout << "z+" << endl;
            }
            else if (state == 1)
            {
                cout << "z-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }

        transfer_matrix = Eigen::AngleAxisd(angle, axis_rotation_along);
        result = transfer_matrix * rotation_matrix.inverse();
        return result;
    }
    const vector<Eigen::Vector3d> getVertices() const{
        return vertices;
    }
    const Vector3d getCenter() const
    {
        return center;
    }
    const Matrix3d getRotation() const
    {
        return rotation_matrix;
    }
    const Vector3d getExtents() const
    {
        return size_vector;
    }
    const Vector3d getRotExtents() const
    {
        Eigen::Vector3d result(0, 0, 0);
        if (use_x)
        {
            if (state == 0)
            {
                // cout<<"x+"<<endl;
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
            }
            else if (state == 1)
            {
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
                // cout<<"x-"<<endl;
                // axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                // angle=M_PI/2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                // cout<<"y+"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else if (state == 1)
            {
                // cout<<"y-"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                // cout<<"z+"<<endl;
                result = size_vector;
            }
            else if (state == 1)
            {
                // cout<<"z-"<<endl;
                result = size_vector;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }
        return result;
    }

    string generate_string_version() const
    {
        string result = "";
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(vertices[i][j]) + ",";
            }
        }
        for (int i = 0; i < 9; i++)
        {
            result = result + to_string(rotation_matrix(i)) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(center[j]) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(size_vector[j]) + ",";
        }

        //for (int i = 0; i < 2; i++)
        //{
        //    for (int j = 0; j < 3; j++)
        //    {
        //        result = result + to_string(global_in_out[i][j]) + ",";
        //    }
        //}
        result = result + to_string(xsize) + ",";
        result = result + to_string(ysize) + ",";
        result = result + to_string(zsize) + ",";
        result = result + to_string(id) + ",";
        result = result + to_string(state) + ",";
        result = result + to_string(volume) + ",";
        result = result + to_string(use_x) + ",";
        result = result + to_string(use_y) + ",";
        result = result + to_string(use_z) + ",";
        // cout<<result<<endl;
        return result;
    }

private:
    vector<Eigen::Vector3d> vertices; 
    double volume = 0;
    Eigen::Vector3d center;                
    Eigen::Matrix3d rotation_matrix;       
    Eigen::Vector3d size_vector;           
    double xsize, ysize, zsize;            
    int id = 0;                            
    //vector<Eigen::Vector3d> global_in_out; 
    int state = 0;                         
    bool use_x = false;                    
    bool use_y = false;                    
    bool use_z = true; 
};

class gcs{
public:
    gcs(){

    }
    gcs(ros::NodeHandlePtr &nh_ptr)
    : nh_ptr(nh_ptr)
    {
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        position_pair["/jurong"] = {Eigen::Vector3d(0, 0, 1), false};
        position_pair["/raffles"] = {Eigen::Vector3d(0, 0, 1), false};
        position_pair["/changi"] = {Eigen::Vector3d(0, 0, 1), false};
        position_pair["/sentosa"] = {Eigen::Vector3d(0, 0, 1), false};
        position_pair["/nanyang"]={Eigen::Vector3d(0, 0, 1), false};


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

        bbox_sub = nh_ptr->subscribe<sensor_msgs::PointCloud>("/gcs/bounding_box_vertices", 10, &gcs::bboxCallback, this);
        agent_position_sub= nh_ptr->subscribe<std_msgs::String>("/broadcast/gcs", 10, &gcs::positionCallback, this);

        Agent_init_ensure_Timer=nh_ptr->createTimer(ros::Duration(1.0 / 10.0),  &gcs::TimerEnsureCB,     this);
        message_publish_timer = nh_ptr->createTimer(ros::Duration(1.0 / 10.0),  &gcs::TimerMessageCB,     this);
    }

private:
    caric_mission::CreatePPComTopic srv;
    ros::ServiceClient client;
    ros::Publisher cmd_pub_;
    bool serviceAvailable = false;
    ros::NodeHandlePtr nh_ptr;
    ros::Subscriber bbox_sub;
    ros::Subscriber agent_position_sub;
    ros::Timer Agent_init_ensure_Timer;
    ros::Timer message_publish_timer;
    double agent_last_update_time;

    vector<Boundingbox> bbox_set;
    map<string,position_info> position_pair;

    bool finish_bbox_record = false;
    bool agent_info_get=false;

    list<string> namelist;

    bool finish_message_generate = false;
    string result;

    vector<vector<Boundingbox>> bbox_set_2_teams;

    void bboxCallback(const sensor_msgs::PointCloud::ConstPtr &msg){
        if(finish_bbox_record||!agent_info_get||finish_message_generate)
        {
            return;
        }
        ROS_INFO("get bbox data!");
        sensor_msgs::PointCloud cloud = *msg;
        int num_points = cloud.points.size();
        if (num_points % 8 == 0 && num_points > 8 * bbox_set.size())
        {
            int num_box = num_points / 8;
            for (int i = 0; i < num_box; i++)
            {
                vector<Eigen::Vector3d> point_vec;
                for (int j = 0; j < 8; j++)
                {
                    point_vec.push_back(Eigen::Vector3d(cloud.points[8 * i + j].x, cloud.points[8 * i + j].y, cloud.points[8 * i + j].z));
                }
                bbox_set.push_back(Boundingbox(point_vec, i));
                point_vec.clear();
            }
        }
        finish_bbox_record = true;


        task_assignment();
        generate_message();
    }

    void positionCallback(const std_msgs::String msg){
        //ROS_INFO_STREAM(msg.data);
        istringstream str(msg.data);
        string type;
        getline(str,type,';');
        if(type=="init_pos")
        {
            //ROS_INFO("got initial pos!");
            string name_space;
            getline(str,name_space,';');
            string position_str;
            getline(str,position_str,';');
            if(!position_pair[name_space].update){
                agent_last_update_time=ros::Time::now().toSec();
                position_pair[name_space].position=str2point(position_str);
                position_pair[name_space].update=true;
            }
        }
    }
    void TimerEnsureCB(const ros::TimerEvent &)
    {   
        //string msg = "finish bbox record: " + btos(finish_bbox_record) + ", agent info get: " + btos(agent_info_get) + ", finish message generation: " + btos(finish_message_generate);
        //ROS_INFO("%s\n",msg.c_str());
        if(agent_info_get)
        {
            return;
        }
        bool finish_agent_record=false; 
        for(auto& name:namelist)
        {
            if(position_pair[name].update){
                finish_agent_record=true;
                break;
            }
        }
        if(!finish_agent_record)
        {
            return;
        }
        double time_now=ros::Time::now().toSec();

        if(fabs(time_now-agent_last_update_time)>10)
        {
            agent_info_get=true;
        }
    }
    void TimerMessageCB(const ros::TimerEvent &){
        if(finish_message_generate)
        {
            std_msgs::String task;
            task.data=result;
            cmd_pub_.publish(task);
            //ROS_INFO("task message sent!");
        }
    }

    void task_assignment(){
        if(!finish_bbox_record) return; 
        bbox_set_2_teams.resize(2);
        for(int i=0;i<bbox_set.size();i++){
            if(i<bbox_set.size()/2){
                bbox_set_2_teams[0].push_back(bbox_set[i]);
            } else {
                bbox_set_2_teams[1].push_back(bbox_set[i]);
            }
        }
        ROS_INFO("task assigned!");

    }

    Eigen::Vector3d str2point(string input)
    {
        Eigen::Vector3d result;
        std::vector<string> value;
        boost::split(value, input, boost::is_any_of(","));
        // cout<<input<<endl;
        if (value.size() == 3)
        {
            result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
        }
        else
        {
            cout << input << endl;
            cout << "error use str2point 2" << endl;
        }
        return result;
    }

    void generate_message()
    {
        ROS_INFO("start generating messages!");
        result="";
        for (int i = 0; i < bbox_set_2_teams.size(); i++)
        {
            for (int j = 0; j < bbox_set_2_teams[i].size(); j++)
            {
                result += bbox_set_2_teams[i][j].generate_string_version()+ ";";
            }
            result += "|";
        }
        finish_message_generate=true;
    }

    string btos(bool x){
        if(x) return "true";
        return "false";
    }

};