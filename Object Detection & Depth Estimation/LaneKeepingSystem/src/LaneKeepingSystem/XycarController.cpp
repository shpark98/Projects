// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file XycarController.cpp
 * @author Hayami099, SeunghanYu, shpark98
 * @brief Xycar Drive Controller Class source file
 * @version 1.1
 * @date 2023-06-29
 */

#include "LaneKeepingSystem/XycarController.hpp"
namespace Xycar {

template <typename PREC>
void XycarController<PREC>::setConfiguration(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();

    mAvoidAngle = config["LIDAR"]["AVOID_ANGLE"].as<PREC>();
    mAvoidDuration1 = config["LIDAR"]["AVOID_DURATION1"].as<PREC>();
    mAvoidDuration2 = config["LIDAR"]["AVOID_DURATION2"].as<PREC>();
    mAvoidSpeed = config["LIDAR"]["AVOID_SPEED"].as<PREC>();

    mLoopCntAngle = config["LIDAR"]["AVOID_LOOPCNT_ANGLE"].as<uint32_t>();
    mLoopCntSide = config["LIDAR"]["AVOID_LOOPCNT_SIDE"].as<uint32_t>();
    mLoopCntForword = config["LIDAR"]["AVOID_LOOPCNT_FOWORD"].as<uint32_t>();

    mDriveAngle = config["TSTL"]["DRIVE_ANGLE"].as<PREC>();

    mIsLaneKeeping = config["XYCAR"]["INIT_IS_LANEKEEPING"].as<bool>();
    mIsStopline = config["XYCAR"]["INIT_IS_STOPLINE"].as<bool>();
    mIsRedLight = config["XYCAR"]["INIT_IS_REDLIGHT"].as<bool>();

    mPublisher = mNodeHandle->advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
}

template <typename PREC>
void XycarController<PREC>::drive(ControllSender sender, PREC AngleOrDistance, PREC XycarSpeed)
{
    if (sender == ControllSender::STOPLINEDETECTOR) {
        if( mIsStopline == true) { 
            std::cout << "stopline start" << std::endl;           
            drive_stopline(AngleOrDistance, XycarSpeed);
            std::cout << "stopline end" << std::endl;            
        }        
    } else if (sender == ControllSender::TSTL_STOP) {
        mIsStopline = true;
        std::cout << "Stopline true" << std::endl;
        return;
    } else if (sender == ControllSender::TSTL_REDLIGHT) {
        mIsStopline = true;
        std::cout << "Stopline true" << std::endl;  
        mIsRedLight = true;
        std::cout << "RedLight true" << std::endl;        
        return;
    } else if (sender == ControllSender::TSTL_GREENLIGHT) {
        mIsStopline = false;
        mIsRedLight = false;
        std::cout << "Stopline false" << std::endl;
        std::cout << "RedLight false" << std::endl;
        mIsLaneKeeping = true;
        return;
    } else if (sender == ControllSender::TSTL_LEFT) {        
        ros::Time now_time = ros::Time::now();
        ros::Duration time_diff =  now_time - mSignTime;
        if(time_diff.toSec() > 3.5 ){
            mIsLaneKeeping = false;
            drive_TSTL_turn(-1, AngleOrDistance);
            mSignTime = ros::Time::now();
        }
        return;
    } else if (sender == ControllSender::TSTL_RIGHT) {
        ros::Time now_time = ros::Time::now();
        ros::Duration time_diff =  now_time - mSignTime;
        if(time_diff.toSec() > 3.5 ){
            mIsLaneKeeping = false;
            drive_TSTL_turn(1, AngleOrDistance);
            mSignTime = ros::Time::now();
        }        
        return;
    }
        
    //std::cout << "XycarController::drive" << steeringAngle << "/" << XycarSpeed << std::endl;
    if( (abs(mPreviousAngle) <= 15 ) ){
        if( sender == ControllSender::LIDAR_LEFT) {
            mIsLaneKeeping = false;
            drive_lidar(1);
        } else if( sender == ControllSender::LIDAR_RIGHT) {
            mIsLaneKeeping = false;
            drive_lidar(-1);
        } else if( sender == ControllSender::LIDAR_STOP) {
            mIsLaneKeeping = false;
            drive_lidar_stop();
        } else if( sender == ControllSender::LIDAR_DRIVE) {
            mIsLaneKeeping = true;
        }
    }

    if( mIsLaneKeeping ){
        xycar_msgs::xycar_motor motorMessage;
        mPreviousAngle = AngleOrDistance;
        motorMessage.angle = std::round(AngleOrDistance);
        motorMessage.speed = std::round(XycarSpeed);
        mPublisher.publish(motorMessage);
    }    
}

template <typename PREC>
void XycarController<PREC>::drive_stopline(PREC steeringAngle, PREC XycarSpeed)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(0);
    motorMessage.speed = std::round(0);
    mPublisher.publish(motorMessage);

    mIsStopline = false;

    std::cout << "1" << std::endl;

    if (mIsRedLight) {
        std::cout << "2" << std::endl;
        mIsLaneKeeping = false;
        return;
    }

    ros::Duration(5.0).sleep();
    
    ros::Time previous_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff =  current_time - previous_time;

    while(time_diff.toSec() < 2)
    {
        motorMessage.angle = std::round(steeringAngle);
        motorMessage.speed = std::round(XycarSpeed);
        mPublisher.publish(motorMessage);
        current_time = ros::Time::now();
        time_diff = current_time - previous_time;
    }
    
}

template <typename PREC>
void XycarController<PREC>::drive_lidar_stop()
{
    xycar_msgs::xycar_motor motorMessage;

    motorMessage.angle = std::round(0);
    motorMessage.speed = std::round(0);
    mPublisher.publish(motorMessage);

    ros::Duration(3.0).sleep();     

    mIsLaneKeeping = true;
}

template <typename PREC>
void XycarController<PREC>::drive_lidar(float objleft)
{
    xycar_msgs::xycar_motor motorMessage;

    float angle1 = mAvoidAngle*objleft;
    float angle2 = -(mAvoidAngle*objleft);

    // turn 01
    ros::Time previous_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff =  current_time - previous_time;

    while(time_diff.toSec() < 2)
    {        
        motorMessage.angle = std::round(angle1);
        motorMessage.speed = std::round(4.0);
        mPublisher.publish(motorMessage);
        current_time = ros::Time::now();
        time_diff = current_time - previous_time;
    }

    previous_time = ros::Time::now();
    current_time = ros::Time::now();
    time_diff =  current_time - previous_time;

    while(time_diff.toSec() < 3.5)
    {        
        motorMessage.angle = std::round(angle2);
        motorMessage.speed = std::round(3.5);
        mPublisher.publish(motorMessage);
        current_time = ros::Time::now();
        time_diff = current_time - previous_time;
    }

    mIsLaneKeeping = true;
}

template <typename PREC>
void XycarController<PREC>::drive_lidar_avoid(int objleft)
{
    // xycar_msgs::xycar_motor motorMessage;

    // float angle1 = mAvoidAngle*objleft;
    // float angle2 = -(mAvoidAngle*objleft);

    // // turn 01
    // int i = 0;
    // for(i=0; i<mLoopCntAngle; i++) {
    //     motorMessage.angle = std::round(angle1);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }
    // for(i=0; i<mLoopCntSide; i++) {
    //     motorMessage.angle = std::round(0);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }

    // // turn 02
    // int i = 0;
    // for(i=0; i<mLoopCntAngle; i++) {
    //     motorMessage.angle = std::round(angle2);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }
    // for(i=0; i<mLoopCntForword; i++) {
    //     motorMessage.angle = std::round(0);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }

    // // turn 03
    // int i = 0;
    // for(i=0; i<mLoopCntAngle; i++) {
    //     motorMessage.angle = std::round(angle2);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }
    // for(i=0; i<mLoopCntSide; i++) {
    //     motorMessage.angle = std::round(0);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }

    // // turn 04
    // int i = 0;
    // for(i=0; i<mLoopCntAngle; i++) {
    //     motorMessage.angle = std::round(angle1);
    //     motorMessage.speed = std::round(mAvoidSpeed);
    //     mPublisher.publish(motorMessage);
    //     ros::Duration(0.5).sleep();    
    // }
    // motorMessage.angle = std::round(0);
    // motorMessage.speed = std::round(mAvoidSpeed);
    // mPublisher.publish(motorMessage);

    
    // mIsLaneKeeping = true;
}

template <typename PREC>
void XycarController<PREC>::drive_TSTL_turn(PREC tstl_direction, PREC distance)
{
    xycar_msgs::xycar_motor motorMessage;

    PREC diff =  distance - 103; // 뒤에 숫자를 키우면 더 빨리 꺾음
    PREC diff_time = diff / 19;
    float angle = mDriveAngle*tstl_direction; // tstl_direction : 우회전 1, 좌회전 -1 -> steeringangle  우회전: 30, 좌회전 : -30


    ros::Time previous_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff =  current_time - previous_time;

    if(distance > 103)
    {
        while(time_diff.toSec() < diff_time)
        {        
            motorMessage.angle = std::round(0);
            motorMessage.speed = std::round(4.0);
            mPublisher.publish(motorMessage);
            current_time = ros::Time::now();
            time_diff = current_time - previous_time;
        }
    }

    previous_time = ros::Time::now();
    current_time = ros::Time::now();
    time_diff =  current_time - previous_time;

    while(time_diff.toSec() < 5.8) // 5.2
    {        
        motorMessage.angle = std::round(angle);
        motorMessage.speed = std::round(4.0);
        mPublisher.publish(motorMessage);
        current_time = ros::Time::now();
        time_diff = current_time - previous_time;
    }

    mIsLaneKeeping = true;

}

template class XycarController<float>;
template class XycarController<double>;
} // namespace Xycar