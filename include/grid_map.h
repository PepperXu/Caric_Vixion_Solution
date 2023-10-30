
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

#include "gcs_controller.h"

struct agent_local
{
    bool in_bounding_box = false;
    bool planning_in_bounding_box = false;
    Eigen::Vector3i position_index;
    Eigen::Vector3i planning_index;
    double time = 0;
    double state = 0;
    double priority = 0;
};


class grid_map
{
public:
    grid_map() {}

    // Function use boundingbox message to build map
    grid_map(Boundingbox box, Eigen::Vector3d grid_size_in, int Teamsize_in, vector<string> team_list)
    {
        local_dict["/jurong"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 5};
        local_dict["/raffles"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 4};
        local_dict["/sentosa"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 3};
        local_dict["/changi"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 2};
        local_dict["/nanyang"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 1};
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        for (auto &name : team_list)
        {
            if (name == "jurong" || name == "raffles")
            {
                continue;
            }
            follower.push_back("/" + name);
        }
        team_size = Teamsize_in;
        fly_in_index = Eigen::Vector3i(0, 0, 0);
        rotation_matrix = box.getSearchRotation();
        rotation_matrix_inv = rotation_matrix.inverse();
        rotation_quat = Eigen::Quaterniond(rotation_matrix_inv);
        map_global_center = box.getCenter();
        map_quat_size = box.getRotExtents();
        grid_size = grid_size_in;
        initial_the_convert();
        interval = floor(map_shape.z() / team_size);
        cout << "Teamsize:"
             << "team_size" << endl; // test
        for (int i = 1; i < Teamsize_in; i++)
        {
            region_slice_layer.push_back(i * interval);
            finish_flag.push_back(0);
            finish_exp_flag.push_back(0);
        }
        set_under_ground_occupied();
    }

    // Function use grid size to build map used in construct the global map
    grid_map(Eigen::Vector3d grid_size_in)
    {
        local_dict["/jurong"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 5};
        local_dict["/raffles"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 4};
        local_dict["/sentosa"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 3};
        local_dict["/changi"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 2};
        local_dict["/nanyang"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 1};
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        fly_in_index = Eigen::Vector3i(0, 0, 0);
        map_global_center = Eigen::Vector3d(0, 0, 0);
        map_quat_size = Eigen::Vector3d(200, 200, 100);
        grid_size = grid_size_in;
        rotation_matrix = Eigen::Matrix3d::Identity();
        rotation_matrix_inv = rotation_matrix.inverse();
        rotation_quat = Eigen::Quaterniond(rotation_matrix_inv);
        initial_the_convert();
        set_under_ground_occupied();
    }

    // Function for update the map and interest point
    void insert_point(Eigen::Vector3d point_in)
    {
        Eigen::Vector3d point_in_local = rotation_matrix * (point_in - map_global_center);
        if (out_of_range(point_in_local, false))
        {
            return;
        }
        Eigen::Vector3i bias_index(0, 0, 0);
        if (fabs(point_in_local.x()) < 0.5 * grid_size.x())
        {
            bias_index.x() = 0;
        }
        else
        {
            if (point_in_local.x() > 0)
            {
                bias_index.x() = floor((point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) + 1;
            }
            else
            {
                bias_index.x() = -floor((-point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) - 1;
            }
        }

        if (fabs(point_in_local.y()) < 0.5 * grid_size.y())
        {
            bias_index.y() = 0;
        }
        else
        {
            if (point_in_local.y() > 0)
            {
                bias_index.y() = floor((point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) + 1;
            }
            else
            {
                bias_index.y() = -floor((-point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) - 1;
            }
        }
        if (fabs(point_in_local.z()) < 0.5 * grid_size.z())
        {
            bias_index.z() = 0;
        }
        else
        {
            if (point_in_local.z() > 0)
            {
                bias_index.z() = floor((point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) + 1;
            }
            else
            {
                bias_index.z() = -floor((-point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) - 1;
            }
        }
        Eigen::Vector3i true_index = bias_index + map_index_center;
        if (map[true_index.x()][true_index.y()][true_index.z()] == 1)
        {
            return;
        }
        else
        {
            map[true_index.x()][true_index.y()][true_index.z()] = 1;
            occupied_num++;
            map_cloud_massage = point3i2str(true_index) + ";" + map_cloud_massage;
            for (int x = true_index.x() - 1; x < true_index.x() + 2; x++)
            {
                for (int y = true_index.y() - 1; y < true_index.y() + 2; y++)
                {
                    for (int z = true_index.z() - 1; z < true_index.z() + 2; z++)
                    {
                        if (out_of_range_index(Eigen::Vector3i(x, y, z)))
                        {
                            continue;
                        }
                        if (abs(x - true_index.x()) + abs(y - true_index.y()) + abs(z - true_index.z()) == 1)
                        {
                            if (map[x][y][z] == 0 && visited_map[x][y][z] == 0)
                            {
                                interest_map[x][y][z] = 1;
                            }
                            else
                            {
                                interest_map[x][y][z] = 0;
                            }
                        }
                    }
                }
            }
            return;
        }
    }

    visualization_msgs::MarkerArray Draw_map()
    {
        visualization_msgs::MarkerArray markers;
        for (int x = 0; x < map_shape.x(); x++)
        {
            for (int y = 0; y < map_shape.y(); y++)
            {
                for (int z = 0; z < map_shape.z(); z++)
                {
                    if (map[x][y][z] == 1)
                    {
                        markers.markers.push_back(generate_marker(Eigen::Vector3i(x, y, z), 0, markers.markers.size()));
                    }
                    else if (interest_map[x][y][z] == 1)
                    {
                        markers.markers.push_back(generate_marker(Eigen::Vector3i(x, y, z), 1, markers.markers.size()));
                    }
                }
            }
        }

        return markers;
    }

    void update_position(Eigen::Vector3d point)
    {
        Eigen::Vector3d point_local = rotation_matrix * (point - map_global_center);
        if (out_of_range(point_local, false))
        {
            in_my_range = false;
            return;
        }
        Eigen::Vector3i index = get_index(point);
        if (now_position_index != index && visited_map[index.x()][index.y()][index.z()] == 0 && search_direction.empty())
        {
            search_direction = get_search_target(index);
            time_start=ros::Time::now().toSec();
        }
        if (search_direction.empty())
        {
            visited_map[index.x()][index.y()][index.z()] = 1;
        }
        if(fabs(ros::Time::now().toSec()-time_start)>3)
        {
            visited_map[index.x()][index.y()][index.z()] = 1;
        }
        now_position_global = point;
        now_position_index = index;
        now_position_local = point_local;
        in_my_range = true;
    }

    Eigen::Vector3i get_index(Eigen::Vector3d point_in)
    {
        Eigen::Vector3d point_in_local = rotation_matrix * (point_in - map_global_center);
        Eigen::Vector3i bias_index(0, 0, 0);
        if (fabs(point_in_local.x()) < 0.5 * grid_size.x())
        {
            bias_index.x() = 0;
        }
        else
        {
            if (point_in_local.x() > 0)
            {
                bias_index.x() = floor((point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) + 1;
            }
            else
            {
                bias_index.x() = -floor((-point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) - 1;
            }
        }

        if (fabs(point_in_local.y()) < 0.5 * grid_size.y())
        {
            bias_index.y() = 0;
        }
        else
        {
            if (point_in_local.y() > 0)
            {
                bias_index.y() = floor((point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) + 1;
            }
            else
            {
                bias_index.y() = -floor((-point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) - 1;
            }
        }

        if (fabs(point_in_local.z()) < 0.5 * grid_size.z())
        {
            bias_index.z() = 0;
        }
        else
        {
            if (point_in_local.z() > 0)
            {
                bias_index.z() = floor((point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) + 1;
            }
            else
            {
                bias_index.z() = -floor((-point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) - 1;
            }
        }
        Eigen::Vector3i result = bias_index + map_index_center;
        return result;
    }
    
    void Astar_local(Eigen::Vector3d target, string myname, string leader_name, bool &flag, bool islong)
    {
        vector<vector<vector<int>>> map_temp = map;
        if (myname == "/jurong" || myname == "/raffles")
        {
            if (true)
            { // Here condition should be whether need waiting;
                for (auto &name : namelist)
                {
                    if (myname == name)
                    {
                        continue;
                    }
                    else
                    {
                        if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1 || true)
                        {
                            if (local_dict[name].in_bounding_box)
                            {
                                Eigen::Vector3i tar = local_dict[name].position_index;
                                map_temp[tar.x()][tar.y()][tar.z()] = 1;
                            }
                            if (local_dict[name].planning_in_bounding_box)
                            {
                                Eigen::Vector3i tar = local_dict[name].planning_index;
                                map_temp[tar.x()][tar.y()][tar.z()] = 1;
                            }
                        }
                    }
                }
                Eigen::Vector3i tar_index = get_index(target);
                list<Eigen::Vector3i> path_tamp;
                if (!islong)
                {
                    path_tamp = astar_planner.get_path(map_temp, now_position_index, tar_index);
                }
                else
                {
                    path_tamp = astar_planner.get_path_long(map_temp, now_position_index, tar_index);
                }

                if (path_tamp.empty())
                {
                    path_final_global = now_position_global;
                    path_index = path_tamp;
                    flag = true;
                }
                else
                {
                    flag = false;
                    path_final_global = target;
                    path_index = path_tamp;
                }
                generate_the_global_path();
                return;
            }
            else
            {
                path_index = {};
                generate_the_global_path();
                return;
            }
        }
        else
        {
            for (auto &name : namelist)
            {
                if (myname == name || name == leader_name)
                {
                    continue;
                }
                else
                {
                    if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1 || true)
                    {
                        if (local_dict[name].in_bounding_box)
                        {
                            Eigen::Vector3i tar = local_dict[name].position_index;
                            map_temp[tar.x()][tar.y()][tar.z()] = 1;
                        }
                        if (local_dict[name].planning_in_bounding_box)
                        {
                            Eigen::Vector3i tar = local_dict[name].planning_index;
                            map_temp[tar.x()][tar.y()][tar.z()] = 1;
                        }
                    }
                }
            }
            Eigen::Vector3i tar_index = get_index(target);
            list<Eigen::Vector3i> path_tamp;
            if (!islong)
            {
                path_tamp = astar_planner.get_path(map_temp, now_position_index, tar_index);
            }
            else
            {
                path_tamp = astar_planner.get_path_long(map_temp, now_position_index, tar_index);
            }
            if (path_tamp.empty())
            {
                path_final_global = now_position_global;
                path_index = path_tamp;
                flag = true;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[leader_name].time) < 1)
                {
                    path_tamp.pop_front();
                }
                flag = false;
                path_final_global = target;
                path_index = path_tamp;
            }
            generate_the_global_path();
            return;
        }
    }
    
    void Astar_photo(Eigen::Vector3d target, string myname, bool &flag)
    {
        vector<vector<vector<int>>> map_temp = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {
                    if (local_dict[name].in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        map_temp[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        map_temp[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i tar_index = get_index(target);
        list<Eigen::Vector3i> path_tamp;
        path_tamp = astar_planner.get_path(map_temp, now_position_index, tar_index);
        if (path_tamp.empty())
        {
            path_final_global = now_position_global;
            path_index = path_tamp;
            flag = true;
        }
        else
        {
            flag = false;
            path_final_global = target;
            path_index = path_tamp;
        }
        generate_the_global_path();
    }
    
    Eigen::Vector3d get_fly_in_point_global()
    {
        // cout<<"fly in output"<<fly_in_index.transpose()<<endl;//test debug
        return get_grid_center_global(fly_in_index);
        // return get_grid_center_global(Eigen::Vector3i(0,0,1));
    }
    
    bool check_whether_fly_in(bool print)
    {
        if (print)
        {
            cout << "now position" << now_position_index.transpose() << endl;
            cout << "fly in" << fly_in_index.transpose() << endl;
        }
        if ((now_position_index - fly_in_index).norm() < 2 && in_my_range)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    void update_fly_in_index(bool replan)
    {
        if (map[fly_in_index.x()][fly_in_index.y()][fly_in_index.z()] == 1 || replan)
        {
            int x = fly_in_index.x();
            int y = fly_in_index.y();
            int z = fly_in_index.z();
            int distance = min({abs(x), abs(y), abs(map_shape.x() - x), abs(map_shape.y() - y)});
            int top;
            int bottom;
            int left;
            int right;
            int i = x;
            int j = y;

            for (int k = z; k < map_shape.z(); k++)
            {
                for (distance; distance <= 3; distance++)
                {
                    top = map_shape.y() - distance;
                    bottom = distance;
                    left = distance;
                    right = map_shape.x() - distance;

                    while (i < right && j == bottom)
                    {
                        if (x == i && y == j && z == k)
                        {
                            i++;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            i++;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                i++;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        i++;
                    }
                    while (j < top && i == right)
                    {
                        if (x == i && y == j && z == k)
                        {
                            j++;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            j++;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                j++;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        j++;
                    }
                    while (i > left && j == top)
                    {
                        if (x == i && y == j && z == k)
                        {
                            i--;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            i--;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                i--;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        i--;
                    }
                    while (j > bottom && i == left)
                    {
                        if (x == i && y == j && z == k)
                        {
                            j--;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            j--;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                j--;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        j--;
                    }
                    i = distance + 1;
                    j = distance + 1;
                }
                cout << "Not find point in layer:" << k << endl;
                distance = 0;
            }
        }
    }
    
    Eigen::Vector3d get_next_point(bool global)
    {
        if (!path_global.empty())
        {
            Eigen::Vector3i index = get_index(path_global.front());
            if (map[index.x()][index.y()][index.z()] == 0)
            {
                return path_global.front();
            }
            else
            {
                return now_position_global;
            }
        }
        else
        {
            if (map[now_position_index.x()][now_position_index.y()][now_position_index.z()] == 1 && global)
            {
                return now_position_global;
            }
            else
            {
                if (get_index(path_final_global) == now_position_index)
                {
                    return path_final_global;
                }
                else
                {
                    return get_grid_center_global(now_position_index);
                }
            }
        }
    }
    
    nav_msgs::Path get_path_show()
    {
        return path_global_show_message;
    }
    
    void set_state(int a)
    {
        mystate = a;
    }
    
    int get_state()
    {
        return mystate;
    }
    
    int get_state_leader()
    {
        bool flag_state = true;
        if (mystate == 3)
        {
            return mystate;
        }
        for (auto &name : follower)
        {
            if (local_dict[name].state != 2)
            {
                flag_state = false;
                return mystate;
            }
        }
        if (flag_state && mystate == 2)
        {
            mystate = 3;
        }
        return mystate;
    }
    
    bool get_whether_pop()
    {
        if (mystate == 3)
        {
            // cout<<"mystate:"<<mystate<<endl;
            // return true;
        }
        if (mystate != 3)
        {
            return false;
        }
        else
        { 
            for (auto &name : follower)
            {
                if (local_dict[name].state != 0)
                {
                    return false;
                }
            }
            if(follower.size()==0&&mystate!=3){
                return false;
            }
        }
        return true;
    }
    
    void exploration_layer(string myname, int region_index)
    {
        list<Eigen::Vector3i> path_index_temp;
        if (is_not_empty())
        {
            path_index_temp = Dijkstra_search_2D_with_3D(height, region_slice_layer[region_index], myname);
        }
        else
        {
            path_index_temp = Dijkstra_search_edge(height, region_slice_layer[region_index], myname);
        }
        if (path_index_temp.empty())
        {
            finish_exp_flag[region_index] = 1;
            if (height < region_slice_layer[region_index])
            {
                height = region_slice_layer[region_index];
            }
            else
            {
                if (local_dict[follower[region_index]].state != 1)
                {
                    finish_exp_flag[region_index] = 1;
                    path_index_temp = Dijkstra_search_fly_in_xy(interval * (region_index - 1), height, myname);
                    if (path_index_temp.empty() || path_index_temp.size() == 1)
                    {
                        path_index_temp = Dijkstra_search_edge(height, region_slice_layer[region_index], myname);
                        if (path_index_temp.empty())
                        {
                            height++;
                        }
                    }
                }
                else
                {
                    finish_flag[region_index] = 1;
                }
            }
        }
        path_index_temp.reverse();
        path_index = path_index_temp;
        generate_the_global_path();
    }
    
    void exploration(string myname)
    {
       ////yolo();
        for (int i = 0; i < finish_flag.size(); i++)
        {
            if (finish_flag[i] == 0)
            {
                exploration_layer(myname, i);
                return;
            }
        }
        take_photo(myname);
    }
    
    void take_photo(string myname)
    {
        if (!init_task_id)
        {
            int i = 0;
            while (i < follower.size())
            {
                if (follower[i] == myname)
                {
                    break;
                }
                i++;
            }
            task_id = i;
        }
        if(follower.size()==0){
            //yolo();
            take_photo_layer(0, map_shape.z()-1, myname);
            //yolo();
            return;
        }


        if (task_id == 0)
        {
            take_photo_layer(0, region_slice_layer[0], myname);
            return;
        }
        else if (task_id == follower.size())
        {
            take_photo_layer(region_slice_layer[task_id - 1], map_shape.z() - 1, myname);
        }
        else
        {
            take_photo_layer(region_slice_layer[task_id - 1], region_slice_layer[task_id], myname);
        }
    }
    
    void take_photo_layer(int low, int high, string myname)
    {
        list<Eigen::Vector3i> path_index_temp;
        if (height < low + 1)
        {
            height = low + 1;
        }
        if (myname == "/raffles" || myname == "/jurong")
        {
            if (finish_flag_leader)
            {

            }
            else
            {
                bool tamp = true;
                if(follower.size()==0){
                    tamp=false;
                }
                for (auto &name : follower)
                {
                    if (local_dict[name].state != 2)
                    {
                        tamp = false;
                        break;
                    }
                }
                finish_flag_leader = tamp;
            }
        }
        else
        {
            finish_flag_leader = false;
        }

        if (height >= high || finish_flag_leader)
        {
            path_index_temp = Dijkstra_search_fly_in_xy(low + 1, high - 1, myname);
            // cout<<"Path size:"<<path_index_temp.size()<<endl;
            if (path_index_temp.empty() || path_index_temp.size() == 1 || finish_flag_leader)
            {
                if (mystate != 3)
                {
                    mystate = 2;
                }
            }
            path_index_temp.reverse();
            path_index = path_index_temp;
            generate_the_global_path();
            return;
        }

        path_index_temp = Dijkstra_search_2D_with_3D(height, high - 1, myname);
        if (path_index_temp.empty())
        {
            if (myname == "/raffles" || myname == "/jurong")
            {
                height += 4;
            }
            else
            {
                height += 4;
            }
        }
        path_index_temp.reverse();
        path_index = path_index_temp;
        generate_the_global_path();
        return;
    }

    bool is_not_empty()
    {
        for (int x = 0; x < map_shape.x(); x++)
        {
            for (int y = 0; y < map_shape.y(); y++)
            {
                if (map[x][y][height] == 1)
                {
                    return true;
                }
            }
        }
        return false;
    }

    void update_gimbal(Eigen::Vector3d direction_global, bool print)
    {
        if (print)
        {
            cout << "direction_global:" << direction_global.transpose() << endl;
            // cout<<"matrix:"<<endl;
            // cout<<Rpy2Rot(direction_global)<<endl;
            if (!search_direction.empty())
            {
                cout << "target:" << search_direction.front().transpose() << endl;
                // cout<<"matrix:"<<endl;
                // cout<<Rpy2Rot(search_direction.front())<<endl;
            }
        }
        if (search_direction.empty())
        {
            return;
        }
        else if ((direction_global - search_direction.front()).norm() < 0.30)
        {
            search_direction.pop_front();
            // cout<<"pop front search"<<endl;// test
        }
    }

    void insert_cloud_from_str(istringstream &msg)
    {
        string number_of_map;
        getline(msg, number_of_map, ';');
        int number_map = stoi(number_of_map);

        while (occupied_num < number_map)
        {
            string index_occupied;
            getline(msg, index_occupied, ';');

            Eigen::Vector3i index_occ_tamp;
            if (str2point3i(index_occupied, index_occ_tamp))
            {
                insert_map_index(index_occ_tamp);
            }
        }
    }

    void get_gimbal_rpy(Eigen::Vector3d &result)
    {
        list<Eigen::Vector3d> search_temp = search_direction;

        if (search_temp.empty())
        {
        }
        else
        {
            result = search_temp.front();
        }
    }
    bool get_mission_finished()
    {
        return is_finished;
    }
    string get_num_str()
    {
        string result;
        result = to_string(occupied_num) + ";";
        return result;
    }
    string get_map_str()
    {
        return map_cloud_massage;
    }
    void set_fly_in_index(string tar)
    {
        try
        {
            Eigen::Vector3i vec_fly;
            if (str2point3i(tar, vec_fly))
            {
                fly_in_index = vec_fly;
            }
        }
        catch (const std::invalid_argument &e)
        {
            cout << "Invalid argument" << e.what() << endl;
            return;
        }
        catch (const std::out_of_range &e)
        {
            cout << "Out of range" << e.what() << endl;
            return;
        }
    }
    string get_fly_in_str()
    {
        return (point3i2str(fly_in_index) + ";");
    }
    void update_local_dict(istringstream &str)
    {
        string name;
        getline(str, name, ';');
        if (name != "/jurong" && name != "/raffles" && name != "/changi" && name != "/sentosa" && name != "/nanyang")
        {
            return;
        }
        else
        {
            string position_str;
            getline(str, position_str, ';');
            agent_local info_temp;
            // cout<<"position_str:"<<position_str<<endl;
            Eigen::Vector3d global_nbr_position_point = str2point(position_str);
            if (!out_of_range_global(global_nbr_position_point, false))
            {
                info_temp.in_bounding_box = true;
                info_temp.position_index = get_index(global_nbr_position_point);
            }
            string path_point;
            getline(str, path_point, ';');
            // cout<<"path_point:"<<path_point<<endl;
            Eigen::Vector3d next_nbr_path_po = str2point(path_point);
            if (!out_of_range_global(next_nbr_path_po, false))
            {
                info_temp.planning_in_bounding_box = true;
                info_temp.planning_index = get_index(next_nbr_path_po);
            }

            info_temp.time = ros::Time::now().toSec();
            info_temp.state = local_dict[name].state;
            info_temp.priority = local_dict[name].priority;
            local_dict[name] = info_temp;
        }
    }
    void update_state(string name, int state_in)
    {
        local_dict[name].state = state_in;
    }
    list<string> get_state_string_list()
    {
        list<string> result;
        for (int i = 0; i < finish_exp_flag.size(); i++)
        {
            if (finish_exp_flag[i] == 1 && finish_flag[i] == 0)
            {
                string str_state_set = follower[i] + ";1;";
                // cout<<"str_state_set:"<<str_state_set<<endl;
                result.push_back(str_state_set);
            }
        }
        return result;
    }

private:
    // communication part
    list<string> namelist;
    std::map<string, agent_local> local_dict;
    int mystate = 0;
    //
    int team_size;
    bool is_finished = false;
    AStar astar_planner;
    double time_start=0;

    bool init_task_id = false;
    int task_id = 0;
    string map_cloud_massage;
    int occupied_num = 0;
    int exploration_state = 0;
    vector<vector<vector<int>>> map;
    vector<vector<vector<int>>> interest_map;
    vector<vector<vector<int>>> visited_map;
    Eigen::Vector3d grid_size;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Matrix3d rotation_matrix_inv;
    Eigen::Quaterniond rotation_quat;
    Eigen::Vector3d map_global_center;
    Eigen::Vector3i map_shape;
    Eigen::Vector3i map_index_center;
    Eigen::Vector3d map_quat_size;
    Eigen::Vector3d now_position_global;
    Eigen::Vector3d now_position_local;
    Eigen::Vector3i now_position_index;
    Eigen::Vector3i fly_in_index;
    Eigen::Vector3d path_final_global;
    list<Eigen::Vector3d> path_global;
    list<Eigen::Vector3i> path_index;
    list<Eigen::Vector3d> search_direction;
    vector<int> region_slice_layer;
    vector<int> finish_flag;
    vector<int> finish_exp_flag;
    vector<string> follower;
    bool Developing = true;
    bool in_my_range = false;
    nav_msgs::Path path_global_show_message;
    int height = 0;
    int interval = 0;
    bool finish_flag_leader = false;
    void initial_the_convert()
    {
        int x_lim;
        int y_lim;
        int z_lim;
        if (map_quat_size.x() < 0.5 * grid_size.x())
        {
            x_lim = 0;
        }
        else
        {
            x_lim = floor((map_quat_size.x() - 0.5 * grid_size.x()) / grid_size.x()) + 1;
        }
        if (map_quat_size.y() < 0.5 * grid_size.y())
        {
            y_lim = 0;
        }
        else
        {
            y_lim = floor((map_quat_size.y() - 0.5 * grid_size.y()) / grid_size.y()) + 1;
        }
        if (map_quat_size.z() < 0.5 * grid_size.z())
        {
            z_lim = 0;
        }
        else
        {
            z_lim = floor((map_quat_size.z() - 0.5 * grid_size.z()) / grid_size.z()) + 1;
        }
        x_lim = x_lim + 1;
        y_lim = y_lim + 1;
        z_lim = z_lim + 1;
        cout << "x lim:" << x_lim << endl;
        cout << "y lim:" << y_lim << endl;
        cout << "z lim:" << z_lim << endl;
        map_shape = Eigen::Vector3i(2 * x_lim + 1, 2 * y_lim + 1, 2 * z_lim + 1);
        cout << "Map shape:" << map_shape.transpose() << endl;
        map_index_center = Eigen::Vector3i(x_lim, y_lim, z_lim);
        cout << "Map Center Index:" << map_index_center.transpose() << endl;
        map = vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
        interest_map = vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
        visited_map = vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
        astar_planner = AStar(map, map_shape);
    }
    void set_under_ground_occupied()
    {
        for (int x = 0; x < map_shape.x(); x++)
        {
            for (int y = 0; y < map_shape.y(); y++)
            {
                for (int z = 0; z < map_shape.z(); z++)
                {
                    Eigen::Vector3d grid_center_global = get_grid_center_global(Eigen::Vector3i(x, y, z));
                    if (grid_center_global.z() < 0.5 * grid_size.z())
                    {
                        map[x][y][z] = 1;
                    }
                }
            }
        }
    }
    // Function whether a local point is out of range
    bool out_of_range(Eigen::Vector3d point, bool out_put)
    {
        if (fabs(point.x()) > (fabs(map_shape.x() / 2) + 0.5) * grid_size.x() || fabs(point.y()) > (fabs(map_shape.y() / 2) + 0.5) * grid_size.y() || fabs(point.z()) > (fabs(map_shape.z() / 2) + 0.5) * grid_size.z())
        {
            if (out_put)
            {
                cout << "xbool:" << (fabs(point.x()) > fabs(map_shape.x() / 2) * grid_size.x() ? "yes" : "no") << endl;
                cout << "xlim:" << fabs(map_shape.x() / 2) * grid_size.x() << endl;
                cout << "ylim:" << fabs(map_shape.y() / 2) * grid_size.y() << endl;
                cout << "grid size:" << grid_size.transpose() << endl;
                cout << "map_shape:" << map_shape.transpose() << endl;
                cout << "center:" << map_index_center.transpose() << endl;
                cout << "out range point" << point.transpose() << endl;
                cout << "Desired point" << (rotation_matrix * (get_grid_center_global(Eigen::Vector3i(0, 0, 0)) - map_global_center)).transpose() << endl;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

        // Function whether a local point is out of range
    bool out_of_range_global(Eigen::Vector3d point_in, bool out_put)
    {
        Eigen::Vector3d point=rotation_matrix*(point_in-map_global_center);
        if (fabs(point.x()) > (fabs(map_shape.x() / 2) + 0.5) * grid_size.x() || fabs(point.y()) > (fabs(map_shape.y() / 2) + 0.5) * grid_size.y() || fabs(point.z()) > (fabs(map_shape.z() / 2) + 0.5) * grid_size.z())
        {
            if (out_put)
            {
                cout << "xbool:" << (fabs(point.x()) > fabs(map_shape.x() / 2) * grid_size.x() ? "yes" : "no") << endl;
                cout << "xlim:" << fabs(map_shape.x() / 2) * grid_size.x() << endl;
                cout << "ylim:" << fabs(map_shape.y() / 2) * grid_size.y() << endl;
                cout << "grid size:" << grid_size.transpose() << endl;
                cout << "map_shape:" << map_shape.transpose() << endl;
                cout << "center:" << map_index_center.transpose() << endl;
                cout << "out range point" << point.transpose() << endl;
                cout << "Desired point" << (rotation_matrix * (get_grid_center_global(Eigen::Vector3i(0, 0, 0)) - map_global_center)).transpose() << endl;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    // Function whether a local point_index is out of range
    bool out_of_range_index(Eigen::Vector3i point)
    {
        if (point.x() >= 0 && point.x() < map_shape.x() && point.y() >= 0 && point.y() < map_shape.y() && point.z() >= 0 && point.z() < map_shape.z())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    // Function whether a local point_index is out of range and whether the z in limitation
    bool out_of_range_index(Eigen::Vector3i point, int top_z, int bottom_z)
    {
        if (top_z <= bottom_z)
        {
            cout << "Error use function" << endl;
        }
        if (point.x() >= 0 && point.x() < map_shape.x() && point.y() >= 0 && point.y() < map_shape.y() && point.z() > bottom_z && point.z() < top_z && point.z() >= 0 && point.z() < map_shape.z())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    // Function to get the grid center point in global
    Eigen::Vector3d get_grid_center_global(Eigen::Vector3i grid_index)
    {
        Eigen::Vector3d bias = (grid_index - map_index_center).cast<double>();
        Eigen::Vector3d local_result = bias.cwiseProduct(grid_size);
        Eigen::Vector3d global_result = rotation_matrix_inv * local_result + map_global_center;
        return global_result;
    }
    // Function to generate map marker
    visualization_msgs::Marker generate_marker(Eigen::Vector3i index, int type, int id)
    {
        // type- 0:occupied 1:interest
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube_marker_array";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector3d grid_center = get_grid_center_global(index);
        marker.pose.position.x = grid_center.x();
        marker.pose.position.y = grid_center.y();
        marker.pose.position.z = grid_center.z();
        marker.pose.orientation.x = rotation_quat.x();
        marker.pose.orientation.y = rotation_quat.y();
        marker.pose.orientation.z = rotation_quat.z();
        marker.pose.orientation.w = rotation_quat.w();
        marker.scale.x = grid_size.x();
        marker.scale.y = grid_size.y();
        marker.scale.z = grid_size.z();
        if (type == 0 && grid_center.z() > 0)
        {
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if (type == 1 && grid_center.z() > 0)
        {
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 0.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        return marker;
    }
    // Function to generate 2D layer search path with 3D Dijkstra
    list<Eigen::Vector3i> Dijkstra_search_2D_with_3D(int layer, int upper, string myname)
    {
        vector<vector<vector<int>>> grid = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {
                    if (local_dict[name].in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        // cout<<"position"<<tar.transpose()<<endl;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        // cout<<"motion"<<tar.transpose()<<endl;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i start = now_position_index;
        vector<Vector3i> directions = {Vector3i(0, 1, 0), Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

        // Initialize the queue and visited flag.
        queue<list<Eigen::Vector3i>> q;
        q.push({start});
        vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
        visited[start.x()][start.y()][start.z()] = true;

        while (!q.empty())
        {
            list<Eigen::Vector3i> path = q.front();
            q.pop();

            Eigen::Vector3i curr = path.back();
            if (interest_map[curr.x()][curr.y()][curr.z()] == 1 && visited_map[curr.x()][curr.y()][curr.z()] == 0 && curr.z() == layer)
            {
                return path; // Found the path, return the complete path.
            }

            for (const auto &dir : directions)
            {
                int nextRow = curr.x() + dir.x();
                int nextCol = curr.y() + dir.y();
                int nextHeight = curr.z() + dir.z();
                if (isValidMove(nextRow, nextCol, nextHeight,grid) && !visited[nextRow][nextCol][nextHeight] && nextHeight <= upper)
                {
                    list<Vector3i> newPath = path;
                    newPath.push_back(Vector3i(nextRow, nextCol, nextHeight));
                    q.push(newPath);
                    visited[nextRow][nextCol][nextHeight] = true;
                }
            }
        }
        // If the path is not found, return an empty list.
        return {};
    }
    // Function to generate 3D layer search/path planning with 3D Dijkstra
    list<Eigen::Vector3i> Dijkstra_search_edge(int layer, int upper, string myname)
    {
        vector<vector<vector<int>>> grid = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {
                    // cout<<"name:"<<name<<endl;
                    if (local_dict[name].in_bounding_box)
                    {
                        // cout<<"pos insert"<<endl;
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        // cout<<"motion insert"<<endl;
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i start = now_position_index;
        vector<Vector3i> directions = {Vector3i(0, 1, 0), Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

        // Initialize the queue and visited flag.
        queue<list<Eigen::Vector3i>> q;
        q.push({start});
        vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
        visited[start.x()][start.y()][start.z()] = true;

        while (!q.empty())
        {
            list<Eigen::Vector3i> path = q.front();
            q.pop();

            Eigen::Vector3i curr = path.back();
            if ((curr.x() == 0 || curr.y() == 0 || curr.x() == map_shape.x() - 1 || curr.y() == map_shape.y() - 1) && visited_map[curr.x()][curr.y()][curr.z()] == 0 && curr.z() == layer)
            {
                return path; // Found the path, return the complete path.
            }

            for (const auto &dir : directions)
            {
                int nextRow = curr.x() + dir.x();
                int nextCol = curr.y() + dir.y();
                int nextHeight = curr.z() + dir.z();
                if (isValidMove(nextRow, nextCol, nextHeight,grid) && !visited[nextRow][nextCol][nextHeight] && nextHeight <= upper)
                {
                    list<Vector3i> newPath = path;
                    newPath.push_back(Vector3i(nextRow, nextCol, nextHeight));
                    q.push(newPath);
                    visited[nextRow][nextCol][nextHeight] = true;
                }
            }
        }
        // If the path is not found, return an empty list.
        return {};
    }
    list<Eigen::Vector3i> Dijkstra_search_fly_in_xy(int lower, int upper, string myname)
    {
        vector<vector<vector<int>>> grid = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {   
                    if (local_dict[name].in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i start = now_position_index;
        vector<Vector3i> directions = {Vector3i(0, 1, 0), Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

        // Initialize the queue and visited flag.
        queue<list<Eigen::Vector3i>> q;
        q.push({start});
        vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
        visited[start.x()][start.y()][start.z()] = true;

        while (!q.empty())
        {
            list<Eigen::Vector3i> path = q.front();
            q.pop();

            Eigen::Vector3i curr = path.back();
            if ((curr.x() == fly_in_index.x() && curr.y() == fly_in_index.y()) && curr.z() >= lower && curr.z() <= upper)
            {
                return path; // Found the path, return the complete path.
            }

            for (const auto &dir : directions)
            {
                int nextRow = curr.x() + dir.x();
                int nextCol = curr.y() + dir.y();
                int nextHeight = curr.z() + dir.z();
                if (isValidMove(nextRow, nextCol, nextHeight,grid) && !visited[nextRow][nextCol][nextHeight] && nextHeight <= upper && nextHeight >= lower)
                {
                    list<Vector3i> newPath = path;
                    newPath.push_back(Vector3i(nextRow, nextCol, nextHeight));
                    q.push(newPath);
                    visited[nextRow][nextCol][nextHeight] = true;
                }
            }
        }
        // If the path is not found, return an empty list.
        return {};
    }

    bool isValidMove(int x, int y, int z)
    {
        if (x >= 0 && x < map_shape.x() && y >= 0 && y < map_shape.y() && z < map_shape.z() && z >= 0)
        {
            if (map[x][y][z] == 1)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return false;
        }
    }
    bool isValidMove(int x,int y,int z,vector<vector<vector<int>>> grid)
    {
        if (x >= 0 && x < map_shape.x() && y >= 0 && y < map_shape.y() && z < map_shape.z() && z >= 0)
        {
            if (grid[x][y][z] == 1)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return false;
        }
    }    

    void generate_the_global_path()
    {
        list<Eigen::Vector3i> path_tamp(path_index);
        list<Eigen::Vector3d> point_global_list_tamp;
        if (path_index.empty())
        {
            path_global.clear();
            return;
        }
        nav_msgs::Path global_path_tamp;

        // if (path_tamp.size() == 1)
        // {
        //     path_tamp.pop_back();
        // }
        // else if (path_tamp.size() >= 2)
        // {
        //     Eigen::Vector3i my_index = path_tamp.back();
        //     path_tamp.pop_back();
        //     if ((now_position_global - get_grid_center_global(path_tamp.back())).norm() > (get_grid_center_global(path_tamp.back()) - get_grid_center_global(my_index)).norm())
        //     {
        //         path_tamp.push_back(my_index);
        //     }
        // }

        path_tamp.pop_back();

        // path_tamp.pop_back();
        global_path_tamp.header.frame_id = "world";
        while (!path_tamp.empty())
        {
            Eigen::Vector3i index_current = path_tamp.back();
            Eigen::Vector3d point_current = get_grid_center_global(index_current);
            if (Developing)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "world";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = point_current.x();
                pose.pose.position.y = point_current.y();
                pose.pose.position.z = point_current.z();
                pose.pose.orientation.w = 1.0;
                global_path_tamp.poses.push_back(pose);
            }
            point_global_list_tamp.push_back(point_current);
            path_tamp.pop_back();
        }
        // point_global_list_tamp.push_back(path_final_global);
        path_global_show_message = global_path_tamp;
        path_global = point_global_list_tamp;
    }
    list<Eigen::Vector3d> get_search_target(Eigen::Vector3i true_index)
    {
        list<Eigen::Vector3d> point_list;
        // cout<<"begin:"<<endl;//test
        for (int x = true_index.x() - 1; x < true_index.x() + 2; x++)
        {
            for (int y = true_index.y() - 1; y < true_index.y() + 2; y++)
            {
                for (int z = true_index.z() - 1; z < true_index.z() + 2; z++)
                {
                    if (out_of_range_index(Eigen::Vector3i(x, y, z)))
                    {
                        continue;
                    }
                    if (abs(x - true_index.x()) + abs(y - true_index.y()) + abs(z - true_index.z()) == 1)
                    {
                        if (map[x][y][z] == 1)
                        {
                            point_list.push_back(get_rpy_limited_global(Eigen::Vector3d(x - true_index.x(), y - true_index.y(), z - true_index.z())));
                            // cout<<(Eigen::Vector3i(x,y,z)-true_index).transpose()<<endl;//test
                            // cout<<get_rpy_limited_global(Eigen::Vector3d(x-true_index.x(),y-true_index.y(),z-true_index.z())).transpose()<<endl;//test
                        }
                    }
                }
            }
        }
        // cout<<"end"<<endl;//test

        return point_list;
    }
    double get_rad(Eigen::Vector3d v1, Eigen::Vector3d v2)
    {
        return atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    }
    Eigen::Vector3d get_rpy_limited_global(Eigen::Vector3d target_direction)
    {
        Eigen::Vector3d global_target_direction = rotation_matrix_inv * target_direction;
        // Eigen::Vector3d global_target_direction=target_direction;
        Eigen::Quaterniond quaternion;
        quaternion.setFromTwoVectors(Eigen::Vector3d(1, 0, 0), global_target_direction);
        Eigen::Matrix3d rotation_matrix_here = quaternion.toRotationMatrix();
        Eigen::Vector3d rpy = Rot2rpy(rotation_matrix_here);
        if (abs(rpy.x() + rpy.z()) < 1e-6 || abs(rpy.x() - rpy.z()) < 1e-6)
        {
            rpy.x() = 0;
            rpy.z() = 0;
        }

        if (rpy.y() > M_PI * 4 / 9)
        {
            rpy.y() = M_PI * 4 / 9;
        }
        else if (rpy.y() < -M_PI * 4 / 9)
        {
            rpy.y() = -M_PI * 4 / 9;
        }
        return rpy;
    }
    Eigen::Vector3d Rot2rpy(Eigen::Matrix3d R)
    {
        // Eigen::Vector3d euler_angles=R.eulerAngles(2,1,0);
        // Eigen:;Vector3d result(euler_angles.z(),euler_angles.y(),euler_angles.x());
        // return result;
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
    Eigen::Matrix3d Rpy2Rot(Eigen::Vector3d rpy)
    {
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        result = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        return result;
    }
    string point3i2str(Eigen::Vector3i point)
    {
        string result;
        result = to_string(point.x()) + "," + to_string(point.y()) + "," + to_string(point.z());
        return result;
    }
    bool str2point3i(string str, Eigen::Vector3i &result)
    {
        std::vector<string> value;
        Eigen::Vector3i result_tamp;
        boost::split(value, str, boost::is_any_of(","));
        if (value.size() == 3)
        {
            // cout<<"str:"<<str<<endl;
            try
            {
                result_tamp = Eigen::Vector3i(stoi(value[0]), stoi(value[1]), stoi(value[2]));
            }
            catch (const std::invalid_argument &e)
            {
                return false;
                cout << "Invalid argument" << e.what() << endl;
            }
            catch (const std::out_of_range &e)
            {
                cout << "Out of range" << e.what() << endl;
                return false;
            }
            result = result_tamp;
            return true;
        }
        else
        {
            return false;
            cout << "error use str2point 3" << endl;
            // cout<<"str:"<<str<<endl;
        }
    }
    void insert_map_index(Eigen::Vector3i true_index)
    {
        map[true_index.x()][true_index.y()][true_index.z()] = 1;
        occupied_num++;
        for (int x = true_index.x() - 1; x < true_index.x() + 2; x++)
        {
            for (int y = true_index.y() - 1; y < true_index.y() + 2; y++)
            {
                for (int z = true_index.z() - 1; z < true_index.z() + 2; z++)
                {
                    if (out_of_range_index(Eigen::Vector3i(x, y, z)))
                    {
                        continue;
                    }
                    if (abs(x - true_index.x()) + abs(y - true_index.y()) + abs(z - true_index.z()) == 1)
                    {
                        if (map[x][y][z] == 0 && visited_map[x][y][z] == 0)
                        {
                            interest_map[x][y][z] = 1;
                        }
                        else
                        {
                            interest_map[x][y][z] = 0;
                        }
                    }
                }
            }
        }
    }
    Eigen::Vector3d str2point(string input)
    {
        Eigen::Vector3d result;
        std::vector<string> value;
        boost::split(value, input, boost::is_any_of(","));
        if (value.size() == 3)
        {
            result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
        }
        else
        {
            // cout<<"error use str2point 4"<<endl;
            result = Eigen::Vector3d(1000, 1000, 1000);
        }
        return result;
    }
};