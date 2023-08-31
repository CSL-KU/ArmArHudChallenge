/**
*    This file is part of OV²SLAM.
*    
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/
#pragma once


//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/msg/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <sophus/se3.hpp>

#include "camera_visualizer.hpp"

#include <iostream>

extern std::shared_ptr<rclcpp::Node> nh;

class RosVisualizer {
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    RosVisualizer(std::shared_ptr<rclcpp::Node> n)
        : cameraposevisual_(1, 0, 0, 1)
    {
        std::cout << "\nROS visualizer is being created...\n";

        pub_image_track_ = n->create_publisher<sensor_msgs::msg::Image>("image_track", 1000);

        pub_vo_traj_ = n->create_publisher<visualization_msgs::msg::Marker>("vo_traj", 1000);
        pub_vo_pose_ = n->create_publisher<geometry_msgs::msg::PoseStamped>("vo_pose", 1000);

        vo_traj_msg_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        vo_traj_msg_.color.a = 1.0;
        vo_traj_msg_.color.r = 0.25;
        vo_traj_msg_.color.g = 1.0;
        vo_traj_msg_.color.b = 0.25;
        vo_traj_msg_.scale.x = 0.02;

        camera_pose_visual_pub_ = n->create_publisher<visualization_msgs::msg::MarkerArray>("cam_pose_visual", 1000);

        cameraposevisual_.setScale(0.1);
        cameraposevisual_.setLineWidth(0.01);

        //pub_point_cloud_ = n->create_publisher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("point_cloud", 1000);
        pub_point_cloud_ = n->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1000);
        
        pub_kfs_traj_ = n->create_publisher<visualization_msgs::msg::Marker>("kfs_traj", 1000);
        pub_kfs_pose_ = n->create_publisher<visualization_msgs::msg::MarkerArray>("local_kfs_window", 1000);

        pub_final_kfs_traj_ = n->create_publisher<visualization_msgs::msg::Marker>("final_kfs_traj", 1000);

        kfs_traj_msg_ = vo_traj_msg_;
        kfs_traj_msg_.color.r = 0.25;
        kfs_traj_msg_.color.g = 0.25;
        kfs_traj_msg_.color.b = 1.;

        final_kfs_traj_msg_ = vo_traj_msg_;
        final_kfs_traj_msg_.color.r = 0.75;
        final_kfs_traj_msg_.color.g = 0.25;
        final_kfs_traj_msg_.color.b = 0.25;
		
		br = std::make_unique<tf2_ros::TransformBroadcaster>(nh);
    }

    void pubTrackImage(const cv::Mat &imgTrack, const double time)
    {
        if( pub_image_track_->get_subscription_count() == 0 ) {
            return;
        }

        std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = rclcpp::Time(time);
        //sensor_msgs::msg::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "rgb8", imgTrack).toImageMsg();
        std::shared_ptr<sensor_msgs::msg::Image> imgTrackMsg = cv_bridge::CvImage(header, "rgb8", imgTrack).toImageMsg();
        pub_image_track_->publish(*imgTrackMsg);
    }

    void pubVO(const Sophus::SE3d &Twc, const double time)
    {   
        // 1. Publish marker message
        // =========================
        vo_traj_msg_.header.stamp = rclcpp::Time(time);
        vo_traj_msg_.header.frame_id = "world";

        geometry_msgs::msg::Point p;
        const Eigen::Vector3d &twc = Twc.translation();
        p.x = twc.x(); p.y = twc.y(); p.z = twc.z();

        if( p.x > 50. || p.y > 50. || p.z > 50 ) {
            if( vo_traj_msg_.scale.x < 0.1 ) {
                vo_traj_msg_.scale.x *= 20;  
                kfs_traj_msg_.scale.x *= 20; 
                final_kfs_traj_msg_.scale.x *= 20;     
            }
        }

        vo_traj_msg_.points.push_back(p);

        pub_vo_traj_->publish(vo_traj_msg_);

        // 2. Publish Pose Stamped + tf
        // ============================
        geometry_msgs::msg::PoseStamped Twc_msg;
        geometry_msgs::msg::Quaternion q;
        const Eigen::Quaterniond eigen_q(Twc.unit_quaternion());

        Twc_msg.pose.position = p;
        q.x = eigen_q.x(); q.y = eigen_q.y();
        q.z = eigen_q.z(); q.w = eigen_q.w();

        Twc_msg.pose.orientation = q;

        Twc_msg.header = vo_traj_msg_.header;

        pub_vo_pose_->publish(Twc_msg);

        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(p.x, p.y, p.z));
        tf2::Quaternion qtf(q.x, q.y, q.z, q.w);
        transform.setRotation(qtf);
		
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = rclcpp::Time(time);
		t.header.frame_id = "world";
		t.child_frame_id = "camera";
		
		t.transform.translation.x = p.x;
		t.transform.translation.y = p.y;
		t.transform.translation.z = p.z;
		
		t.transform.rotation.x = qtf.x();
		t.transform.rotation.y = qtf.y();
		t.transform.rotation.z = qtf.z();
		t.transform.rotation.w = qtf.w();
		br->sendTransform(t);

        // 3. Publish camera visual
        // =========================
        cameraposevisual_.reset();
        cameraposevisual_.add_pose(twc, eigen_q);
        cameraposevisual_.setImageBoundaryColor(1, 0, 0);
        cameraposevisual_.setOpticalCenterConnectorColor(1, 0, 0);
        cameraposevisual_.publish_by(camera_pose_visual_pub_, Twc_msg.header);

        // if( vo_traj_msg_.points.size() >= 3600 ) {
        //     size_t nbpts = vo_traj_msg_.points.size();
        //     std::vector<geometry_msgs::Point> vtmp;
        //     // if( nbpts / 20 < 3600 ) {
        //         vtmp.reserve(nbpts / 20);
        //         for( size_t i = 0 ; i < nbpts ; i+=20 ) {
        //             vtmp.push_back(vo_traj_msg_.points.at(i));
        //         }
        //     // } 
        //     // else {
        //     //     vtmp.reserve(nbpts / 100);
        //     //     for( size_t i = 0 ; i < nbpts ; i+=100 ) {
        //     //         vtmp.push_back(vo_traj_msg_.points.at(i));
        //     //     }
        //     // }

        //     vo_traj_msg_.points.swap(vtmp);
        // }

        return;
    }

    void addVisualKF(const Sophus::SE3d &Twc) 
    {
        const Eigen::Quaterniond eigen_q(Twc.unit_quaternion());
        CameraPoseVisualization kfposevisual(0, 0, 1, 1);
        kfposevisual.add_pose(Twc.translation(), eigen_q);
        kfposevisual.setImageBoundaryColor(0, 0, 1);
        kfposevisual.setOpticalCenterConnectorColor(0, 0, 1);
        vkeyframesposevisual_.push_back(kfposevisual);
    }

    void pubVisualKFs(const double time) 
    {
        if( pub_kfs_pose_->get_subscription_count() == 0 ) {
            return;
        }

        std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = rclcpp::Time(time);

        visualization_msgs::msg::MarkerArray markerArray_msg;

        int j = 0;

        for( auto &camposeviz : vkeyframesposevisual_ )
        {
            for( auto &marker : camposeviz.m_markers )
            {
                marker.header = header;
                marker.id += j;
                markerArray_msg.markers.push_back(marker);
            }
            j++;
        }

        pub_kfs_pose_->publish(markerArray_msg);

        vkeyframesposevisual_.clear();
    }

    void pubPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud, const double time) 
    {
        if( pub_point_cloud_->get_subscription_count() == 0 ) {
            return;
        }
		
		std::shared_ptr<sensor_msgs::msg::PointCloud2> pcloud2 = 
			std::make_shared<sensor_msgs::msg::PointCloud2>();
		pcl::toROSMsg(*pcloud, *pcloud2);
		pcloud2->header.frame_id = "world";
		pcloud2->header.stamp = rclcpp::Time(time);

        /*std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = rclcpp::Time(time);
        pcloud->header = pcl_conversions::toPCL(header);*/
		
        pub_point_cloud_->publish(*pcloud2);
    }

    void addKFsTraj(const Sophus::SE3d &Twc)
    {
        geometry_msgs::msg::Point p;
        const Eigen::Vector3d twc = Twc.translation();
        p.x = twc.x(); p.y = twc.y(); p.z = twc.z();

        kfs_traj_msg_.points.push_back(p);
    }

    void clearKFsTraj()
    {   
        kfs_traj_msg_.points.clear();
    }

    void pubKFsTraj(const double time)
    {   
        if( pub_kfs_traj_->get_subscription_count() == 0 ) {
            return;
        }

        kfs_traj_msg_.header.stamp = rclcpp::Time(time);
        kfs_traj_msg_.header.frame_id = "world";

        pub_kfs_traj_->publish(kfs_traj_msg_);
    }

    void pubFinalKFsTraj(const Sophus::SE3d &Twc, const double time)
    {   
        if( pub_final_kfs_traj_->get_subscription_count() == 0 ) {
            return;
        }

        final_kfs_traj_msg_.header.stamp = rclcpp::Time(time);
        final_kfs_traj_msg_.header.frame_id = "world";

        geometry_msgs::msg::Point p;
        const Eigen::Vector3d twc = Twc.translation();
        p.x = twc.x(); p.y = twc.y(); p.z = twc.z();

        final_kfs_traj_msg_.points.push_back(p);

        pub_final_kfs_traj_->publish(final_kfs_traj_msg_);

        return;
    }

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_image_track_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_vo_traj_;
	std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pub_vo_pose_;
    visualization_msgs::msg::Marker vo_traj_msg_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> camera_pose_visual_pub_;
    CameraPoseVisualization cameraposevisual_;

    //std::shared_ptr<rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> pub_point_cloud_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_kfs_pose_;
    std::vector<CameraPoseVisualization> vkeyframesposevisual_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_kfs_traj_;
	std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_final_kfs_traj_;

    visualization_msgs::msg::Marker kfs_traj_msg_, final_kfs_traj_msg_;
	
	std::unique_ptr<tf2_ros::TransformBroadcaster> br;
};