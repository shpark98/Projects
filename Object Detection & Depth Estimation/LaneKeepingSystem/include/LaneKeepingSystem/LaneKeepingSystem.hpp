// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.hpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class header file
 * @version 1.1
 * @date 2023-05-02
 */
#ifndef LANE_KEEPING_SYSTEM_HPP_
#define LANE_KEEPING_SYSTEM_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <vector>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"
#include "LaneKeepingSystem/MovingAverageFilter.hpp"
#include "LaneKeepingSystem/PIDController.hpp"

#include "LaneKeepingSystem/LiDARDetector.hpp"
#include "LaneKeepingSystem/XycarController.hpp"
#include "LaneKeepingSystem/TSTLDetector.hpp"

namespace Xycar {
/**
 * @brief Lane Keeping System for searching and keeping Hough lines using Hough, Moving average and PID control
 *
 * @tparam Precision of data
 */
template <typename PREC>
class LaneKeepingSystem
{
public:
    using Ptr = LaneKeepingSystem*;                                     ///< Pointer type of this class
    using ControllerPtr = typename PIDController<PREC>::Ptr;            ///< Pointer type of PIDController
    using FilterPtr = typename MovingAverageFilter<PREC>::Ptr;          ///< Pointer type of MovingAverageFilter
    using DetectorPtr = typename HoughTransformLaneDetector<PREC>::Ptr; ///< Pointer type of LaneDetector
    using LiDARCheckerPtr = typename LiDARDetector<PREC>::Ptr;          ///< Pointer type of LiDARDetector
    using TSTLDetectorPtr = typename TSTLDetector<PREC>::Ptr;     ///< Pointer type of TSTLDetector
    using XycarControllerPtr = typename XycarController<PREC>::Ptr;     ///< Pointer type of XycarController

    static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
    static constexpr double kFrameRate = 33.0;               ///< Frame rate
    /**
     * @brief Construct a new Lane Keeping System object
     */
    LaneKeepingSystem();

    /**
     * @brief Destroy the Lane Keeping System object
     */
    virtual ~LaneKeepingSystem();

    /**
     * @brief Run Lane Keeping System
     */
    void run();
    void testKey();

private:
    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
     */
    void setParams(const YAML::Node& config);

    /**
     * @brief Control the speed of xycar
     *
     * @param[in] steeringAngle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
     */
    void speedControl(PREC steeringAngle, bool check);

    /**
     * @brief publish the motor topic message
     *
     * @param[in] steeringAngle Angle to steer xycar actually
     */
    void drive(PREC steeringAngle);
    
    /**
     * @brief Callback function for image topic
     *
     * @param[in] message Image topic message
     */
    void imageCallback(const sensor_msgs::Image& message);

private:
    ControllerPtr mPID;                      ///< PID Class for Control
    FilterPtr mMovingAverage;                ///< Moving Average Filter Class for Noise filtering
    DetectorPtr mHoughTransformLaneDetector; ///< Hough Transform Lane Detector Class for Lane Detection
    XycarControllerPtr mXycarController;     ///< Xycar Controller Class 
    TSTLDetectorPtr mTSTLDetector;     ///< Object Detector Class 

    // ROS Variables
    ros::NodeHandle* mNodeHandler;          ///< Node Hanlder for ROS. In this case Detector and Controler    
    ros::Subscriber mSubscriber;           ///< Subscriber to receive image    
    std::string mSubscribedTopicName;      ///< Topic name to subscribe
    uint32_t mQueueSize;                   ///< Max queue size for message    

    // LiDAR Check Variables
    LiDARCheckerPtr mLiDARChecker;    

    // OpenCV Image processing Variables
    cv::Mat mFrame; ///< Image from camera. The raw image is converted into cv::Mat

    // Xycar Device variables
    PREC mXycarSpeed;                 ///< Current speed of xycar
    PREC mXycarMaxSpeed;              ///< Max speed of xycar
    PREC mXycarMinSpeed;              ///< Min speed of xycar
    PREC mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
    PREC mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    PREC mDecelerationStep;           ///< How much would deaccelrate xycar depending on threshold

    PREC mNonZeroCnt;
    int mSleepTime;

    // Debug Flag
    bool mDebugging; ///< Debugging or not
    PREC beforeSteeringAngle = 0;
};
} // namespace Xycar

#endif // LANE_KEEPING_SYSTEM_HPP_
