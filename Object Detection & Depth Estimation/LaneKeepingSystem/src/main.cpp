// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file main.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Chihyeon Lee
 * @brief Lane Keeping System Main Function using Hough Transform
 * @version 1.1
 * @date 2023-05-02
 */
#include <iostream>
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

using PREC = float;

int32_t main(int32_t argc, char** argv)
{
    std::cout << "-----------re------------" << std::endl;
    std::cout << "---------------------test02" << std::endl;
    std::cout << "correction 0.3, P 0.25 D 0.1" << std::endl;

    ros::init(argc, argv, "Lane Keeping System");
    Xycar::LaneKeepingSystem<PREC> laneKeepingSystem;

    
    ros::Time previous_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff =  current_time - previous_time;

    while(time_diff.toSec() < 5)
    {
        current_time = ros::Time::now();
        time_diff = current_time - previous_time;
    }


    laneKeepingSystem.run();

    return 0;
}
