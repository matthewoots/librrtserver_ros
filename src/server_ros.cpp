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

/** @brief Construct the search path from RRT search and from its shortened path */
void rrt_server_ros::generate_search_path(pcl::PointCloud<pcl::PointXYZ>::Ptr obs)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    double radii;
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
}

/** @brief Extract the direct goal for the planner that is within the search sphere */
void rrt_server_ros::extract_direct_goal()
{
    double radii;
    radii = search_radius;
    previous_search_points.clear();

    // Find the intersection between any of the legs and the sphere perimeter
    int intersection_idx = -1;
    for (int i = 0; i < (int)global_search_path.size()-1; i++)
    {
        // outside : previous_search_points[i]
        // inside : previous_search_points[i-1]
        if (inside_sphere_check(global_search_path[i], current_control_point, search_radius) &&
            !inside_sphere_check(global_search_path[i+1], current_control_point, search_radius))
        {
            intersection_idx = i+1;
            std::cout << "[server_ros]" << KBLU <<
                " intersection found at idx: " << intersection_idx << 
                KNRM << std::endl;
            break;
        }
    }

    /** @brief Print intersection points wiht sphere */
    // std::cout << "start and end global_search_path check [" << intersection_idx-1 <<
    //     " and " << intersection_idx << "] out of " <<
    //     global_search_path.size() << std::endl;

    // We have reached the goal if failed counts is same as global_search_path size
    if (intersection_idx < 0)
    {
        direct_goal = end;
        for (int i = 1; i < (int)global_search_path.size(); i++)
        {
            previous_search_points.push_back(global_search_path[i]);
        }
        previous_search_points.push_back(end);
        return;
    }

    // direction is inside to outside
    Eigen::Vector3d vect1 = global_search_path[intersection_idx] - global_search_path[intersection_idx-1];
    Eigen::Vector3d vect1_direction = vect1 / vect1.norm();
    // direction is center to inside
    Eigen::Vector3d vect2 = global_search_path[intersection_idx-1] - current_control_point;
    double a = pow(vect1.x(), 2) + pow(vect1.y(), 2) + pow(vect1.z(), 2);
    double b = 2 * (vect1.x() * vect2.x() + 
        vect1.y() * vect2.y() + 
        vect1.z() * vect2.z());
    double c = pow(current_control_point.x(), 2) +
        pow(current_control_point.y(), 2) +
        pow(current_control_point.z(), 2) +
        pow(global_search_path[intersection_idx-1].x(), 2) +
        pow(global_search_path[intersection_idx-1].y(), 2) +
        pow(global_search_path[intersection_idx-1].z(), 2) -
        2 * (global_search_path[intersection_idx-1].x() * current_control_point.x() +
        global_search_path[intersection_idx-1].y() * current_control_point.y() +
        global_search_path[intersection_idx-1].z() * current_control_point.z()) -
        pow(radii,2);

    double u_p = (-b + sqrt(pow(b,2) - 4 * a * c)) / (2 * a);

    Eigen::Vector3d query1 = vect1 * u_p + global_search_path[intersection_idx-1];

    direct_goal = query1;

    // Save previous global_search_path into previous_search_points
    for (int i = 0; i < (int)global_search_path.size(); i++)
    {
        /** @brief Push back the intersection points as the final points */
        // if (intersection_idx-1 == i)
        // {
        //     previous_search_points.push_back(query1);
        //     break;
        // }

        // Do not add the start point
        if (i == 0)
            continue;

        previous_search_points.push_back(global_search_path[i]);
    }

    return; 

}

/** @brief Use this function wisely, since check and update may cause an infinite loop
 * If a bad data is given to the rrt node and it cannot complete the validity check
*/
void rrt_server_ros::check_and_update_search(
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs,  Eigen::Vector3d first_cp)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);
    std::lock_guard<std::mutex> search_points_lock(search_points_mutex);

    if (previous_search_points.empty())
    {
        std::cout << KRED <<
            "[server_ros] previous_search_points.size() is empty" <<
            KNRM << std::endl;
        return;
    }

    // Check to see whether the new control point and the previous inputs
    // have any pointclouds lying inside
    if (!ru.check_line_validity_with_pcl(
        previous_search_points[0], first_cp, obs_threshold*1.1, obs))
    {
        previous_search_points.clear();
        std::cout << KRED <<
            "[server_ros] connecting point is not obs free" <<
            KNRM << std::endl;
        return;
    }

    int last_safe_idx = -1;
    int initial_size = (int)previous_search_points.size();
    for (int i = 1; i < initial_size; i++)
    {
        if (!ru.check_line_validity_with_pcl(
            previous_search_points[i], previous_search_points[i-1], obs_threshold*1.1, obs))
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

    std::cout << "[server_ros] previous_search_points.size(): " << 
        KGRN << previous_search_points.size() << KNRM << std::endl;
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

    check_and_update_search(local_cloud, current_control_point);

    bool bypass = false;
    ros::Time global_start_time = ros::Time::now();
    // Check to see whether the previous data extents to the end, keep this data for checks and to recycle
    if (!previous_search_points.empty())
    {
        /** @brief With valid search points and there is no collision 
         * from check_and_update_search(), hence the waypoints can be reused */ 
        if ((previous_search_points[previous_search_points.size()-1]
            - end).norm() < 0.0001)
        {
            std::lock_guard<std::mutex> pose_lock(pose_update_mutex);
            std::cout << KGRN <<
                "[server_ros] bypass extract direct goal" <<
                KNRM << std::endl;
            bypass = true;
            global_search_path.clear();
            global_search_path.push_back(current_control_point);
            for (int i = 0; i < previous_search_points.size(); i++)
            {
                global_search_path.push_back(previous_search_points[i]);
            }
        }
    }
    
    /** @brief When there is collision or when the search does not extent to the end */ 
    if (!bypass)
        generate_search_path(local_cloud);
    
    extract_direct_goal();
    
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
    // find vector of travel
    double step_size = 0.5;
    bool delete_vector = false;
    Eigen::Vector3d global_vector = Eigen::Vector3d::Zero();
    for (int i = 1; i < global_search_path.size(); i++)
    {
        Eigen::Vector3d vect2 = global_search_path[i] - current_control_point;
        Eigen::Vector3d vect2_dir = vect2 / vect2.norm();

        if (vect2.norm() < step_size * 0.9)
        {
            delete_vector = true;
            continue;
        }

        global_vector = vect2_dir;
        break;
    }
    if (delete_vector)
        previous_search_points.erase(previous_search_points.begin());

    current_control_point += global_vector * step_size;
    std::cout << "current_control_point [" << KBLU << 
        current_control_point.transpose() << "]" << KNRM << std::endl;
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = current_control_point.x();
    pose.pose.position.y = current_control_point.y();
    pose.pose.position.z = current_control_point.z();
    pose_pub.publish(pose);

    visualize_points(0.5, search_radius*2);
    std::cout << std::endl;
}