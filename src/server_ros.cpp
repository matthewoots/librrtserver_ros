/*
* server_ros.cpp
*
* ---------------------------------------------------------------------
* Copyright (C) 2022 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include "server_ros.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace std;
using namespace Eigen;
using namespace rrt_server;

void rrt_server_ros::pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    full_cloud = pcl2_converter(*msg);

    std::lock_guard<std::mutex> cloud_lock(cloud_mutex);
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    local_cloud = ru.pcl_ptr_box_crop(full_cloud, current_control_point, _local_map_size_xyz);
    last_pcl_msg = ros::Time::now();
    local_pcl_pub.publish(local_cloud);
    return;
}

void rrt_server_ros::extract_direct_goal(pcl::PointCloud<pcl::PointXYZ>::Ptr obs)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    double radii;
    // If end goal is inside the search sphere
    // if ((end - current_control_point).norm() < search_radius)
    //     radii = (end - current_control_point).norm();
    // If end goal is outside the search sphere
    // else
        radii = search_radius;

    // Find a RRT path that is quick and stretches to the end point
    vector<Eigen::Vector3d> path = fe_rrt_server.find_rrt_path(
        previous_search_points, obs, current_control_point, end, radii);
    // If path gives an invalid value, execute some form of emergency
    if (path.empty())
    {
        return;
    }

    // Save global_path
    global_search_path.clear();
    global_search_path = path;

    // Find the intersection between any of the legs and the sphere perimeter
    int intersection_idx = -1;
    for (int i = 0; i < (int)path.size()-1; i++)
    {
        // outside : previous_search_points[i]
        // inside : previous_search_points[i-1]
        if (inside_sphere_check(path[i], current_control_point, search_radius) &&
            !inside_sphere_check(path[i+1], current_control_point, search_radius))
        {
            intersection_idx = i+1;
            break;
        }
    }

    /** @brief Print intersection points wiht sphere */
    // std::cout << "start and end path check [" << intersection_idx-1 <<
    //     " and " << intersection_idx << "] out of " <<
    //     path.size() << std::endl;

    // We have reached the goal if failed counts is same as path size
    if (intersection_idx < 0)
    {
        direct_goal = end;
        return;
    }

    // direction is inside to outside
    Eigen::Vector3d vect1 = path[intersection_idx] - path[intersection_idx-1];
    Eigen::Vector3d vect1_direction = vect1 / vect1.norm();
    // direction is center to inside
    Eigen::Vector3d vect2 = path[intersection_idx-1] - current_control_point;
    double a = pow(vect1.x(), 2) + pow(vect1.y(), 2) + pow(vect1.z(), 2);
    double b = 2 * (vect1.x() * vect2.x() + 
        vect1.y() * vect2.y() + 
        vect1.z() * vect2.z());
    double c = pow(current_control_point.x(), 2) +
        pow(current_control_point.y(), 2) +
        pow(current_control_point.z(), 2) +
        pow(path[intersection_idx-1].x(), 2) +
        pow(path[intersection_idx-1].y(), 2) +
        pow(path[intersection_idx-1].z(), 2) -
        2 * (path[intersection_idx-1].x() * current_control_point.x() +
        path[intersection_idx-1].y() * current_control_point.y() +
        path[intersection_idx-1].z() * current_control_point.z()) -
        pow(radii,2);

    double u_p = (-b + sqrt(pow(b,2) - 4 * a * c)) / (2 * a);

    Eigen::Vector3d query1 = vect1 * u_p + path[intersection_idx-1];

    direct_goal = query1;

    // Save previous path
    previous_search_points.clear();
    for (int i = 1; i < (int)path.size(); i++)
    {
        if (intersection_idx-1 == i)
        {
            previous_search_points.push_back(query1);
            break;
        }
        previous_search_points.push_back(path[i]);
    }

    return; 

}

void rrt_server_ros::check_and_update_search(
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);
    std::lock_guard<std::mutex> search_points_lock(search_points_mutex);

    if (previous_search_points.empty())
        return;

    int last_safe_idx = -1;
    int initial_size = (int)previous_search_points.size();
    for (int i = 1; i < initial_size; i++)
    {
        if (!ru.check_line_validity_with_pcl(
            previous_search_points[i], previous_search_points[i-1], obs_threshold, obs))
        {
            last_safe_idx = i-1;
            break;
        }
    }

    if (last_safe_idx >= 0)
    {
        for (int i = last_safe_idx + 1; i < initial_size; i++)
            previous_search_points.erase(previous_search_points.end());
    }

    // int new_size = (int)previous_search_points.size();
    // double nearest_distance = DBL_MAX;
    // int idx = -1; 
    // for (int i = 0; i < new_size; i++)
    // {
    //     double distance = (previous_search_points[i]-current_control_point).norm();
    //     if (distance < nearest_distance)
    //     {
    //         nearest_distance = distance;
    //         idx = i;
    //     }
    // }

    // for (int i = idx-1; i >= 0; i--)
    //     previous_search_points.erase(previous_search_points.begin());
}

void rrt_server_ros::run_search_timer(const ros::TimerEvent &)
{
    if ((ros::Time::now() - last_pcl_msg).toSec() > 2.0)
        return;

    std::lock_guard<std::mutex> cloud_lock(cloud_mutex);

    fe_rrt_server.reset_parameters(
        vector<Eigen::Vector4d>(),
        min_height, max_height, obs_threshold,
        _sub_runtime_error, _runtime_error);

    check_and_update_search(local_cloud);

    ros::Time global_start_time = ros::Time::now();
    extract_direct_goal(local_cloud);
    ros::Duration global_time = ros::Time::now() - global_start_time;
    
    nav_msgs::Path global_path = vector_3d_to_path(global_search_path);

    std::cout << "[server_ros] global_time " << 
        KGRN << global_time.toSec() << KNRM << "s" << std::endl;

    sensor_msgs::PointCloud2 object_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud;
    object_cloud = fe_rrt_server.get_debug_local_pcl();
    pcl::toROSMsg(*object_cloud.get(), object_msg);
    object_msg.header.frame_id = "world";
    
    g_rrt_points_pub.publish(global_path);
    debug_pcl_pub.publish(object_msg);

    // Project the point forward
    Eigen::Vector3d global_vector = global_search_path[1] - global_search_path[0];
    global_vector = global_vector / global_vector.norm();
    current_control_point += global_vector * 0.5;
    std::cout << "current_control_point [" << KBLU << 
        current_control_point.transpose() << "]" << KNRM << std::endl;
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = current_control_point.x();
    pose.pose.position.y = current_control_point.y();
    pose.pose.position.z = current_control_point.z();
    pose_pub.publish(pose);

    visualize_points(0.5, search_radius*2);
}