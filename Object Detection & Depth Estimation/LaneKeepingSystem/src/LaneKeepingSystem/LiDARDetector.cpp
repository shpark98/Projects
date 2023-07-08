// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LiDARDetector.cpp
 * @brief LiDAR Collision Checker Class source file
 * @version 1.1
 * @date 2023-06-29
 */

#include "LaneKeepingSystem/LiDARDetector.hpp"
namespace Xycar {

template <typename PREC>
void LiDARDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mDistanceMeter = config["LIDAR"]["DISTANCE_M"].as<PREC>();
    mThreshold = config["LIDAR"]["CHECK_THRESHOLD"].as<uint32_t>();
    mCheckAngle = config["LIDAR"]["CHECK_ANGLE"].as<uint32_t>();

    mLiDARSubscribedTopicName = config["TOPIC"]["LIDAR_SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mLiDARSubscriber = mNodeHandle->subscribe(mLiDARSubscribedTopicName, mQueueSize, &LiDARDetector::lidarCallback, this);
}
template <typename PREC>
void LiDARDetector<PREC>::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    /*
    int count = scan->scan_time / scan->time_increment;
    //std::cout << "lidarCallback : " << scan->scan_time << " / " << scan->time_increment << std::endl;
    for(int i = 0; i < count; i++) {
        // float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if (scan->ranges[i] != 0 && scan->ranges[i] <= mDistanceMeter)
        {
            std::cout << "range" << i << ":" << scan->ranges[i] << std::endl;
        }
    }
    */
    
    
    reciveScan(scan);
}

template <typename PREC>
void LiDARDetector<PREC>::reciveScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int obs_c = 0;
    int obs_l = 0;
    int obs_r = 0;

    for (int i = 0; i <= mCheckAngle; i++)
    {
        if (scan->ranges[i] != 0 && scan->ranges[i] <= mDistanceMeter)
        {
            obs_c++;
        }
    }
    for (int i = (505-mCheckAngle); i <= 505; i++)
    {
        if (scan->ranges[i] != 0 && scan->ranges[i] <= mDistanceMeter)
        {
            obs_c++;
        }
    }     
    int right_e = (505-mCheckAngle);
    int right_s = right_e - (mCheckAngle*4);
    for (int i = right_s; i <= right_e; i++)
    {
        if (scan->ranges[i] != 0 && scan->ranges[i] <= mDistanceMeter)
        {
            obs_r++;
        }
    }
    int left_s = mCheckAngle;
    int left_e = left_s + (mCheckAngle*4);
    for (int i = left_s; i <= left_e; i++)
    {
        if (scan->ranges[i] != 0 && scan->ranges[i] <= mDistanceMeter)
        {
            obs_l++;
        }
    }

    //std::cout << "c: " << obs_c << ", l: " << obs_l << ", r: " << obs_r << std::endl;

    if (obs_c > mThreshold)
    {
        //mStateCollision = StateColision::STOP;
        std::cout << "LiDAR Stop!!" << std::endl;
        mXycarController->drive(ControllSender::LIDAR_STOP, 0, 0);
    }
    else
    {
        if (obs_r > mThreshold)
        {
            //mStateCollision = StateColision::TURN_LEFT;
            std::cout << "LiDAR Right!!" << std::endl;
            mXycarController->drive(ControllSender::LIDAR_RIGHT, 3, 3);
            std::cout << "LiDAR Right End!!" << std::endl;

        }
        else if (obs_l > mThreshold)
        {
            //mStateCollision = StateColision::TURN_RIGHT;
            std::cout << "LiDAR Left!!" << std::endl;
            mXycarController->drive(ControllSender::LIDAR_LEFT, 0, 0);
            std::cout << "LiDAR Left End!!" << std::endl;
        }
        else
        {
            //mStateCollision = StateColision::DRIVE;
            //mXycarController->drive(ControllSender::LIDAR_DRIVE, 0, 0);
            //std::cout << "LiDAR Drive!!" << std::endl;
        }
    }
}

template class LiDARDetector<float>;
template class LiDARDetector<double>;
} // namespace Xycar