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

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <console_bridge/console.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "ov2slam.hpp"
#include "slam_params.hpp"

typedef std::shared_ptr<const sensor_msgs::msg::Image> ImageConstPtr;

std::shared_ptr<rclcpp::Node> nh;

class SensorsGrabber {

public:
    SensorsGrabber(SlamManager *slam) : pslam_(slam) {
        std::cout << "\nSensors Grabber is created...\n";
    }

    void subLeftImage(const sensor_msgs::msg::Image::SharedPtr image) const
	{
        std::lock_guard<std::mutex> lock(img_mutex);
        img0_buf.push(image);
    }

    void subRightImage(const sensor_msgs::msg::Image::SharedPtr image) const
	{
        std::lock_guard<std::mutex> lock(img_mutex);
        img1_buf.push(image);
    }

    cv::Mat getGrayImageFromMsg(const sensor_msgs::msg::Image::SharedPtr &img_msg)
    {
        // Get and prepare images
        cv_bridge::CvImageConstPtr ptr;
        try {    
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        } 
        catch(cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp_action"), "\n\n\ncv_bridge exeception: %s\n\n\n", e.what());
        }

        return ptr->image;
    }

    // extract images with same timestamp from two topics
    // (mostly derived from Vins-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
    void sync_process()
    {
        std::cout << "\nStarting the measurements reader thread!\n";
        
        while( !pslam_->bexit_required_ )
        {
            if( pslam_->pslamstate_->stereo_ )
            {
                cv::Mat image0, image1;

                std::lock_guard<std::mutex> lock(img_mutex);

                if (!img0_buf.empty() && !img1_buf.empty())
                {
                    double time0 = (double)img0_buf.front()->header.stamp.sec + 1e-9*(double)img0_buf.front()->header.stamp.nanosec;
                    double time1 = (double)img1_buf.front()->header.stamp.sec + 1e-9*(double)img0_buf.front()->header.stamp.nanosec;

                    // sync tolerance
                    if(time0 < time1 - 0.015)
                    {
                        img0_buf.pop();
                        std::cout << "\n Throw img0 -- Sync error : " << (time0 - time1) << "\n";
                    }
                    else if(time0 > time1 + 0.015)
                    {
                        img1_buf.pop();
                        std::cout << "\n Throw img1 -- Sync error : " << (time0 - time1) << "\n";
                    }
                    else
                    {
                        image0 = getGrayImageFromMsg(img0_buf.front());
                        image1 = getGrayImageFromMsg(img1_buf.front());
                        img0_buf.pop();
                        img1_buf.pop();

                        if( !image0.empty() && !image1.empty() ) {
                            pslam_->addNewStereoImages(time0, image0, image1);
                        }
                    }
                }
            } 
            else if( pslam_->pslamstate_->mono_ ) 
            {
                cv::Mat image0;

                std::lock_guard<std::mutex> lock(img_mutex);

                if ( !img0_buf.empty() )
                {
                    double time = (double)img0_buf.front()->header.stamp.sec;
                    image0 = getGrayImageFromMsg(img0_buf.front());
                    img0_buf.pop();

                    if( !image0.empty()) {
                        pslam_->addNewMonoImage(time, image0);
                    }
                }
            }

            std::chrono::milliseconds dura(1);
            std::this_thread::sleep_for(dura);
        }

        std::cout << "\n Bag reader SyncProcess thread is terminating!\n";
    }

    mutable std::queue<sensor_msgs::msg::Image::SharedPtr> img0_buf;
    mutable std::queue<sensor_msgs::msg::Image::SharedPtr> img1_buf;
    mutable std::mutex img_mutex;
    
    SlamManager *pslam_;
};


int main(int argc, char** argv)
{
    // Init the node
    //ros::init(argc, argv, "ov2slam_node");
	rclcpp::init(argc, argv);
	nh = rclcpp::Node::make_shared("ov2slam_node");

    if(argc < 2)
    {
       std::cout << "\nUsage: rosrun ov2slam ov2slam_node parameters_files/params.yaml\n";
       return 1;
    }

    std::cout << "\nLaunching OV²SLAM...\n\n";

    //ros::NodeHandle nh("~");
	//std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("ov2slam_node");
	

    // Load the parameters
    std::string parameters_file = argv[1];

    std::cout << "\nLoading parameters file : " << parameters_file << "...\n";

    const cv::FileStorage fsSettings(parameters_file.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
       std::cout << "Failed to open settings file...";
       return 1;
    } else {
        std::cout << "\nParameters file loaded...\n";
    }

    std::shared_ptr<SlamParams> pparams;
    pparams.reset( new SlamParams(fsSettings) );

    // Create the ROS Visualizer
    std::shared_ptr<RosVisualizer> prosviz;
    prosviz.reset( new RosVisualizer(nh) );

    // Setting up the SLAM Manager
    SlamManager slam(pparams, prosviz);

    // Start the SLAM thread
    std::thread slamthread(&SlamManager::run, &slam);

    // Create the Bag file reader & callback functions
    SensorsGrabber sb(&slam);

    // Create callbacks according to the topics set in the parameters file
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subleft = 
		nh->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 2, 
			std::bind(&SensorsGrabber::subLeftImage, &sb, std::placeholders::_1));
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subright = 
		nh->create_subscription<sensor_msgs::msg::Image>("/cam1/image_raw", 2, 
			std::bind(&SensorsGrabber::subRightImage, &sb, std::placeholders::_1)); //, &sb);

    // Start a thread for providing new measurements to the SLAM
    std::thread sync_thread(&SensorsGrabber::sync_process, &sb);

    // ROS Spin
    rclcpp::spin(nh);

    // Request Slam Manager thread to exit
    slam.bexit_required_ = true;

    // Waiting end of SLAM Manager
    while( slam.bis_on_ ) {
        std::chrono::seconds dura(1);
        std::this_thread::sleep_for(dura);
    }

    return 0;
}
