// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file XycarController.hpp
 * @author Hayami099, SeunghanYu, shpark98
 * @brief Xycar Drive Controller Class header file
 * @version 1.1
 * @date 2023-06-29
 */
#ifndef XYCAR_CONTROLLER_HPP_
#define XYCAR_CONTROLLER_HPP_

#include <ros/ros.h>
#include <xycar_msgs/xycar_motor.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

#include <iostream>

namespace Xycar {
/**
 * @brief State of Control
 */
enum class ControllSender : uint8_t
{
    LANEKEEPING_DRIVE= 0,        ///< Controll Message received from LaneKeepingSystem
    LANEKEEPING_STOP= 1,        ///< Controll Message received from LaneKeepingSystem
    LANEKEEPING_BACK= 2,        ///< Controll Message received from LaneKeepingSystem
    LIDAR_DRIVE = 3,              ///< Controll Message received from LiDARDetector
    LIDAR_STOP = 4,              ///< Controll Message received from LiDARDetector
    LIDAR_LEFT = 5,              ///< Controll Message received from LiDARDetector
    LIDAR_RIGHT = 6,              ///< Controll Message received from LiDARDetector
    STOPLINEDETECTOR = 7,       ///< Controll Message received from StopLineDetector
    TSTL_STOP = 8,              ///< Controll Message received from TSTLDetector
    TSTL_LEFT = 9,              ///< Controll Message received from TSTLDetector
    TSTL_RIGHT = 10,            ///< Controll Message received from TSTLDetector
    TSTL_REDLIGHT = 11,
    TSTL_GREENLIGHT = 12,         ///< Controll Message received from TSTLDetector
};

template <typename PREC>
class XycarController
{
public:
    using Ptr = XycarController*; ///< Pointer type of this class
    /**
     * @brief Construct a new XycarController object
     */
    XycarController(const YAML::Node& config, ros::NodeHandle* nodeHandle) 
    { 
        mNodeHandle = nodeHandle; 
        setConfiguration(config); 
        mSignTime = ros::Time::now();
    }
    void drive(ControllSender sender, PREC AngleOrDistance, PREC XycarSpeed);

private:
    ros::NodeHandle* mNodeHandle;
    void setConfiguration(const YAML::Node& config);
    ros::Publisher mPublisher;             ///< Publisher to send message about
    std::string mPublishingTopicName;      ///< Topic name to publish
    xycar_msgs::xycar_motor mMotorMessage; ///< Message for the motor of xycar
    uint32_t mQueueSize;                   ///< Max queue size for message
    PREC mAvoidAngle;                      ///< AvoidAngle
    PREC mAvoidDuration1;                   ///< AvoidDuration1
    PREC mAvoidDuration2;                   ///< AvoidDuration2
    PREC mAvoidSpeed;                      ///< AvoidSpeed
    bool mIsLaneKeeping;
    int mLoopCntAngle;
    int mLoopCntSide;
    int mLoopCntForword;
    PREC mPreviousAngle = 0;               ///< AvoidAngle
    bool mIsStopline;
    bool mIsRedLight;
    PREC mDriveAngle;
    ros::Time mSignTime;

    void drive_stopline(PREC steeringAngle, PREC XycarSpeed);
    void drive_lidar_stop();
    void drive_lidar(float objleft);
    void drive_lidar_avoid(int objleft);
    void drive_TSTL_turn(PREC tstl_direction, PREC distance);
};
} // namespace Xycar
#endif // XYCAR_CONTROLLER_HPP_
