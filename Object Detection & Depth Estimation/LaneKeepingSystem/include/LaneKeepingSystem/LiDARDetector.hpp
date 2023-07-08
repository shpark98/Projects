// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LiDARDetector.hpp
 * @author Hayami099
 * @brief LiDAR Collision Checker Class header file
 * @version 1.1
 * @date 2023-06-29
 */
#ifndef LIDAR_DETECTOR_HPP_
#define LIDAR_DETECTOR_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <stdint.h>
#include <yaml-cpp/yaml.h>

#include "LaneKeepingSystem/XycarController.hpp"

namespace Xycar {

template <typename PREC>
class LiDARDetector
{
public:
    using Ptr = LiDARDetector*; ///< Pointer type of this class
    using XycarControllerPtr = typename XycarController<PREC>::Ptr;     ///< Pointer type of XycarController

    LiDARDetector(const YAML::Node& config, ros::NodeHandle* nodeHandle, XycarControllerPtr XycarController) 
    { 
        mNodeHandle = nodeHandle; 
        mXycarController = XycarController; 
        setConfiguration(config); 
    }
    void reciveScan(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
    ros::NodeHandle* mNodeHandle;
    XycarControllerPtr mXycarController;     ///< Xycar Controller Class  
    void setConfiguration(const YAML::Node& config);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in);
    int mThreshold;
    PREC mDistanceMeter;
    int mCheckAngle;
    
    ros::Subscriber mLiDARSubscriber;      ///< Subscriber to LiDAR message
    std::string mLiDARSubscribedTopicName; ///< Topic name to subscribe
    uint32_t mQueueSize;                   ///< Max queue size for message    
};
} // namespace Xycar
#endif // LIDAR_DETECTOR_HPP_
