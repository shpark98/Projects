// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class source file
 * @version 1.1
 * @date 2023-05-02
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"
#include <cmath>

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    mNodeHandler = new ros::NodeHandle();
    std::string configPath;
    mNodeHandler->getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mHoughTransformLaneDetector = new HoughTransformLaneDetector<PREC>(config);
    setParams(config);  

    mSubscriber = mNodeHandler->subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);

    mXycarController = new XycarController<PREC>(config, mNodeHandler);    
    mLiDARChecker = new LiDARDetector<PREC>(config, mNodeHandler, mXycarController);

    mTSTLDetector = new TSTLDetector<PREC>(config, mNodeHandler, mXycarController);    
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();

    mNonZeroCnt = config["STOP"]["NON_ZERO_COUNT"].as<PREC>();
    mSleepTime = config["STOP"]["SLEEP_TIME"].as<uint32_t>();
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mNodeHandler;
    delete mPID;
    delete mMovingAverage;
    delete mHoughTransformLaneDetector;
    delete mLiDARChecker;
    delete mXycarController;
    delete mTSTLDetector;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    // for video writer
    int fourcc = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
    cv::VideoWriter outputVideo("output.avi", fourcc, kFrameRate, cv::Size(640, 480));

    ros::Rate rate(kFrameRate);

    bool stopDetection = false;
    bool pStopDetection = false;

    while (ros::ok())
    {
        ros::spinOnce();
        if (mFrame.empty())
            continue;
        

        // Stop Detection
        int x = 250;    // Width Start
        int y = 365;    // Height Start
        int width = 125;    // Width Size
        int height = 15;    // Height Size
     
        cv::Mat mFramegray;
        cv::Rect roi(x, y, width, height);
        cv::cvtColor(mFrame, mFramegray, cv::COLOR_BGR2GRAY);
        cv::Mat roiStopDetection = mFramegray(roi);
        cv::Mat binaryroiStopDetection;
        
        cv::threshold(roiStopDetection, binaryroiStopDetection, 128, 255, cv::THRESH_BINARY_INV);
        int nonZeroCount = cv::countNonZero(binaryroiStopDetection);
        // std::cout << "non-zero 픽셀 개수: " << nonZeroCount << std::endl;
        
        pStopDetection = stopDetection;
        stopDetection= false;
        if(nonZeroCount >= mNonZeroCnt){
            stopDetection = true;
        }

        const auto [leftPositionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame);
        mTSTLDetector->setTSTLFrame(mFrame);
            
        int32_t middlePositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);

        // // Position Correction
        bool check = false;
        if (leftPositionX == 0 && rightPositionX == 640){
            check = true;
        }
        else{
            if (leftPositionX == 0) {
                middlePositionX -= static_cast<int32_t>((640 - rightPositionX) * 0.3);
            }
            else if (rightPositionX == 640) {
                middlePositionX += static_cast<int32_t>(leftPositionX * 0.3);
            }
        }

        mMovingAverage->addSample(middlePositionX);

        // mMovingAverage->addSample(static_cast<int32_t>((leftPositionX + rightPositionX) / 2));

        int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());

        int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
        PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));

        mPID->saveAngle(steeringAngle);

        speedControl(steeringAngle, check);

        // std::cout << " steering 각도: " << steeringAngle << std::endl;
        // std::cout << " Before stopDetection : " << stopDetection << std::endl;
        // std::cout << " Before pStopDetection : " << pStopDetection << std::endl;
        // std::cout << " Before steeringAngle : " << steeringAngle << std::endl;
        // std::cout << " " << std::endl;
        if(check){
            mXycarController->drive(ControllSender::LANEKEEPING_BACK, 0, -4);
                // std::cout << " xycar back " << std::endl;
                // std::cout << " Back stopDetection : " << stopDetection << std::endl;
                // std::cout << " Back pStopDetection : " << pStopDetection << std::endl;
                // std::cout << " Back steeringAngle : " << steeringAngle << std::endl;
                // std::cout << " " << std::endl;
                usleep(mSleepTime);
        } else{
            if(stopDetection && (pStopDetection == false) && (abs(steeringAngle) <= 10 )){                
                mXycarController->drive(ControllSender::STOPLINEDETECTOR, steeringAngle, mXycarSpeed);
            }else{
                mXycarController->drive(ControllSender::LANEKEEPING_DRIVE, steeringAngle, mXycarSpeed);
                // std::cout << " xycar go " << std::endl;
                // std::cout << " Go stopDetection : " << stopDetection << std::endl;
                // std::cout << " Go pStopDetection : " << pStopDetection << std::endl;
                // std::cout << " Go steeringAngle : " << steeringAngle << std::endl;
                // std::cout << " " << std::endl;
            }
        }

        if(!check){
            beforeSteeringAngle = steeringAngle;
        }

        if (mDebugging)
        {
            // std::cout << "lpos: " << leftPositionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << std::endl;
            mHoughTransformLaneDetector->drawRectangles(leftPositionX, rightPositionX, estimatedPositionX);
            // cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());

            // save video
            cv::Mat outputframe;
            outputframe = mHoughTransformLaneDetector->getDebugFrame();
            outputVideo << outputframe;
            cv::imshow("output.avi", outputframe);
            cv::waitKey(1);
        }
        // rate.sleep();
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::testKey()
{
    char key;
    std::cin >> key;
    switch (key)
    {
        case 'a':
            mXycarController->drive(ControllSender::LIDAR_LEFT, 0, 0);
            break;
        case 's':
            mXycarController->drive(ControllSender::LIDAR_RIGHT, 0, 0);
            break;
        default:
            break;
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle, bool check)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= (mDecelerationStep * (abs(steeringAngle) / mXycarSpeedControlThreshold));
        //mXycarSpeed -= (mDecelerationStep * (exp(abs(steeringAngle)/20) + pow(1.3, abs(steeringAngle)/20)));

        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    if (check) {
        mXycarSpeed = 5;
        check = false;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
    mHoughTransformLaneDetector -> saveSpeed(mXycarSpeed);

}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    mXycarController->drive(ControllSender::LANEKEEPING_DRIVE, steeringAngle, mXycarSpeed);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
