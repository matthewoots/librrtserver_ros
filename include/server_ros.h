/*
* server_ros.h
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
#ifndef SERVER_ROS_H
#define SERVER_ROS_H

#include "rrt_server_module.h"

#include <string>
#include <thread>   
#include <mutex>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>

#include "rrt_server_module.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace rrt_server;

class rrt_server_ros
{
    private:

        std::mutex search_points_mutex, pose_update_mutex, cloud_mutex;

        ros::NodeHandle _nh;
        ros::Subscriber pcl2_msg_sub;
        ros::Publisher local_pcl_pub, l_rrt_points_pub, g_rrt_points_pub, debug_pcl_pub;

        double _runtime_error, _sub_runtime_error, _search_interval;
        double min_height, max_height;
        double search_radius, obs_threshold;

        pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;

        Eigen::Vector3d start, end, current_control_point;
        Eigen::Vector3d _local_map_size_xyz;
        Eigen::Vector3d direct_goal;

        vector<Eigen::Vector3d> previous_search_points;
        vector<Eigen::Vector3d> global_search_path;

        vector<Eigen::Vector4d> no_fly_zone;

        ros::Time last_pcl_msg;

        rrt_server::rrt_utility ru; 
        rrt_server::rrt_server_node fe_rrt_server;

        ros::Timer search_timer;

        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    
        void extract_direct_goal(pcl::PointCloud<pcl::PointXYZ>::Ptr obs);

        void check_and_update_search(pcl::PointCloud<pcl::PointXYZ>::Ptr obs);

        void generate_search_path(pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pcl);

        void run_search_timer(const ros::TimerEvent &);

    public:

        rrt_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            /** @brief ROS Params */
            _nh.param<double>("runtime_error", _runtime_error, 0.1);
            _nh.param<double>("sub_runtime_error", _sub_runtime_error, 0.02);  
            _nh.param<double>("search_radius", search_radius, 5.0);
            _nh.param<double>("threshold", obs_threshold, 0.5); 
            _nh.param<double>("search_interval", _search_interval, 0.5); 

            std::vector<double> start_list, end_list, local_map_size_list, height_list;

            _nh.getParam("start_position", start_list);
            _nh.getParam("end_position", end_list);
            _nh.getParam("local_map_size_xyz", local_map_size_list);
            _nh.getParam("height", height_list);

            for(int i = 0; i < (int)local_map_size_list.size(); i++) 
                _local_map_size_xyz[i] = local_map_size_list[i];
 
            min_height = height_list[0];
            max_height = height_list[1];

            start = Eigen::Vector3d(start_list[0], start_list[1], start_list[2]);
            end = Eigen::Vector3d(end_list[0], end_list[1], end_list[2]);

            std::cout << "start and end positions \n[" << start.transpose() <<
                 "] [" << end.transpose() << "]" << std::endl;

            pcl2_msg_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
                "/mock_map", 1,  boost::bind(&rrt_server_ros::pcl2_callback, this, _1));

            /** @brief For debug */
            local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
            l_rrt_points_pub = _nh.advertise<nav_msgs::Path>("/rrt_points_local", 10);
            g_rrt_points_pub = _nh.advertise<nav_msgs::Path>("/rrt_points_global", 10);
            debug_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/debug_map", 10);

            /** @brief Timer that when to search */
		    search_timer = _nh.createTimer(
                ros::Duration(_search_interval), 
                &rrt_server_ros::run_search_timer, this, false, false);

            current_control_point = start;
            search_timer.start();
        }


        ~rrt_server_ros()
        {
            search_timer.stop();
        }

        /** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            pcl2_converter(sensor_msgs::PointCloud2 _pc)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(_pc, pcl_pc2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
            
            return tmp_cloud;
        }

        bool inside_sphere_check(Eigen::Vector3d point, 
            Eigen::Vector3d sphere_center, double radii)
        {
            // get the distance vector
            Eigen::Vector3d dv = sphere_center - point;
            double a = 1.0; 
            double b = 1.0; 
            double inv_a2 = 1 / a / a; 
            double inv_b2 = 1 / b / b; 
            // get the ellipsoidal distance 
            double e_d = sqrt(pow(dv.z(),2) * inv_a2 + 
                (pow(dv.x(),2) + pow(dv.y(),2)) * inv_b2); 
            // get the distance error
            double d_e = radii - e_d; 
            if (d_e > 0)
                return true;
            else 
                return false;
        }

        nav_msgs::Path vector_3d_to_path(vector<Vector3d> path_vector)
        {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "world";
            for (int i = 0; i < path_vector.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = path_vector[i][0];
                pose.pose.position.y = path_vector[i][1];
                pose.pose.position.z = path_vector[i][2];
                path.poses.push_back(pose);
            }

            return path;
        }
        
};

#endif
