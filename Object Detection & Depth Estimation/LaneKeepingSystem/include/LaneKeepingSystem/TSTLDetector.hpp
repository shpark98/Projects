/**
 * @file TSTLDetector.hpp
 * @author SeunghanYu
 * @brief Object Detector Class header file
 * @version 1.1
 * @date 2023-06-29
 */
#ifndef TSTL_DETECTOR_HPP_
#define TSTL_DETECTOR_HPP_

#include <ros/ros.h>
#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"
#include "opencv2/opencv.hpp"
#include <iostream>

#include "LaneKeepingSystem/XycarController.hpp"

namespace Xycar {

template <typename PREC>
class TSTLDetector
{
public:
    using Ptr = TSTLDetector*; ///< Pointer type of this class
    using XycarControllerPtr = typename XycarController<PREC>::Ptr;     ///< Pointer type of XycarController
    TSTLDetector(const YAML::Node& config, ros::NodeHandle* nodeHandle, XycarControllerPtr XycarController) 
    { 
        mNodeHandle = nodeHandle;         
        setConfiguration(config); 
        mXycarController = XycarController; 

		findExtrinsicMat();
		initBEV();
        cv::imshow("BEV", mBEV);
	    cv::waitKey(1);
    }
    /**
     * @brief Callback image
     *
     * @param[in] image
     */
    void setTSTLFrame(const cv::Mat& image);

private:
    /**
     * @brief Callback function for BoundingBoxes topic
     *
     * @param[in] message BoundingBoxes topic message
     */
    void detectionCallback(const yolov3_trt_ros::BoundingBoxes& message);

    /**
     * @brief Traffic light detection function
     *
     * @param[in] traffic_xmin, traffic_ymin, traffic_xmax, traffic_ymax
     */
    bool trafficlightDetection(int traffic_xmin, int traffic_ymin, int traffic_xmax, int traffic_ymax);

private:        
    void setConfiguration(const YAML::Node& config);
    void findExtrinsicMat();
	void initBEV();
	void drawObjectRect(std::string clsname, std::vector<cv::Point2f> pts_object);

    float mBEVBlockSize;
	cv::Mat mExtrinsicMat;
	cv::Mat mBEV;

    cv::Mat mTSTLFrame;
    cv::Rect cal_roi;
    cv::Mat mapx;
    cv::Mat mapy;

    ros::NodeHandle* mNodeHandle;
    uint32_t mQueueSize; ///< Max queue size for message
    ros::Subscriber mDetectionSubscriber;
    std::string mDetectionSubscribedTopicName = "/yolov3_trt_ros/detections";
    XycarControllerPtr mXycarController;     ///< Xycar Controller Class  

    int xmin;
    int ymin;
    int xmax;
    int ymax;
    int object_class = -1;

    PREC distance; // unit [cm]
    int mDistanceLowThreshold;
    int mDistanceHighThreshold;
    
    int traffic_xmin;
    int traffic_ymin;
    int traffic_xmax;
    int traffic_ymax;
    bool redlight_on = false;
    
};
} // namespace Xycar

#endif // TSTL_DETECTOR_HPP_
