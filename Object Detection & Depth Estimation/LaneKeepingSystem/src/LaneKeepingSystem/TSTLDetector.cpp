// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file TSTLDetector.cpp
 * @author SeunghanYu
 * @brief Object Detector Class source file
 * @version 1.1
 * @date 2023-06-29
 */

#include "LaneKeepingSystem/TSTLDetector.hpp"
#include <unistd.h>

namespace Xycar {

template <typename PREC>
void TSTLDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
	mBEVBlockSize = config["TSTL"]["BEV_BLOCK_SIZE"].as<uint32_t>();
    mDistanceLowThreshold = config["TSTL"]["DISTANCE_LOW_THRESHOLD"].as<uint32_t>(); // [cm]
	mDistanceHighThreshold = config["TSTL"]["DISTANCE_HIGH_THRESHOLD"].as<uint32_t>(); // [cm]
    mDetectionSubscriber = mNodeHandle->subscribe(mDetectionSubscribedTopicName, mQueueSize, &TSTLDetector::detectionCallback, this);
	 // Calibrate mTSTLFrame
	cv::Mat mtx = (cv::Mat_<PREC>(3, 3) << 374.313954, 0.000000, 313.884148,
										  0.000000, 375.505020, 257.925602,
										  0.000000, 0.000000, 1.000000);
	cv::Mat dist = (cv::Mat_<PREC>(1, 5) << -0.315565, 0.076723, 0.001097, 0.002779, 0.0);
	cv::Mat cal_mtx;
	cal_mtx = cv::getOptimalNewCameraMatrix(mtx, dist, cv::Size(640, 480), 1, cv::Size(640, 480), &cal_roi);
	cv::initUndistortRectifyMap(mtx, dist, cv::Mat(), cal_mtx, cv::Size(640, 480), CV_32FC1, mapx, mapy);
}

template <typename PREC> 
void TSTLDetector<PREC>::setTSTLFrame(const cv::Mat& image)
{
   
	cv::Mat remap_img;
	cv::remap(image, remap_img, mapx, mapy, cv::INTER_LINEAR);

	int x_ = cal_roi.x;
	int y_ = cal_roi.y;
	int w_ = cal_roi.width;
	int h_ = cal_roi.height;
	cv::Mat remap_img_roi = remap_img(cv::Rect(x_, y_, w_, h_));

	cv::Mat calibrated_frame;
	cv::resize(remap_img_roi, calibrated_frame, cv::Size(640, 480));

	mTSTLFrame = calibrated_frame;
}

template <typename PREC>
void TSTLDetector<PREC>::detectionCallback(const yolov3_trt_ros::BoundingBoxes& message)
{
    initBEV();
    
    // all BoundingBox container
	std::vector<std::array<int, 5>> bbox_container;
	
    for (auto& bbox: message.bounding_boxes)
    {
        xmin = bbox.xmin * 640 / 416;
        ymin = bbox.ymin * 480 / 416;
        xmax = bbox.xmax * 640 / 416;
        ymax = bbox.ymax * 480 / 416;
        object_class = bbox.id;

        std::array<int, 5> temparr = {object_class, xmin, ymin, xmax, ymax};
		bbox_container.push_back(temparr);     

		const char* strClass[] = {"left", "right", "stop", "crosswalk", "traffic_light", "xycar"};	

		// std::cout << "detectionCallback: " << object_class << " (" << xmin << "," << ymin << ") (" << xmax << "," << ymax << ")" << std::endl;
		std::vector<cv::Point2f> pts;
		float xcenter = (xmin + xmax) / 2;
		pts.push_back(cv::Point2f(xcenter, ymax));
		drawObjectRect(strClass[object_class], pts);        
    }            

	if (bbox_container.size())
	{
		for (int i = 0; i < bbox_container.size(); ++i) 
		{
			// depth estimation
			/*
				[ x' ]	   [[-3.68147138e-06 -7.57830055e-05 -1.52297332e-01]     [ xmiddle ]
				[ y' ]  =   [ 7.10387051e-04 -9.01524294e-06 -2.26397641e-01]  *  [  ymax   ]
				[ z' ]      [-3.25722568e-06 -3.86141454e-03  1.00000000e+00]]    [    1    ]
				
				xmiddle = (bbox_container[i][1] + bbox_container[i][3]) / 2
				ymax = bbox_container[i][4]
				distance = (x' / z') * 100
				distance unit : [cm]
			*/
			int xmiddle = (bbox_container[i][1] + bbox_container[i][3]) / 2;
			PREC x_ = (-3.68147138e-06 * xmiddle) + (-7.57830055e-05 * bbox_container[i][4]) -1.52297332e-01;
			PREC y_ = (7.10387051e-04 * xmiddle) + (-9.01524294e-06 * bbox_container[i][4]) -2.26397641e-01;
			PREC z_ = (-3.25722568e-06 * xmiddle) + (-3.86141454e-03 * bbox_container[i][4]) + 1;
			PREC cross_distance = (y_ / z_) * 100;
			distance = (x_ / z_) * 100;

			// if distance detection range out
			if (distance <= mDistanceLowThreshold || distance >= mDistanceHighThreshold)
			{
				continue;
			}

			// if cross_distance range out
			if (cross_distance < -65 || cross_distance > 65)
			{
				std::cout << "cross_distance" << cross_distance << std::endl;
				continue;
			}


			/*
			object_class
				0 : left
				1 : right
				2 : stop
				3 : crosswalk
				4 : traffic_light
				5 : xycar
			*/

			// if traffic sign
			if (bbox_container[i][0] == 0)
			{
				std::cout << "TSTL Left!!" << std::endl;
				std::cout << "distance" << distance << std::endl;
				std::cout << "xmin, ymin, xmax, ymax: " << bbox_container[i][1] << " " << bbox_container[i][2] << " " << bbox_container[i][3] << " " << bbox_container[i][4] << std::endl;
				std::cout << "Box Width: " << bbox_container[i][3] - bbox_container[i][1] << std::endl;
				std::cout << "Box Height: " << bbox_container[i][4] - bbox_container[i][2] << std::endl;
				std::cout << "-------------------------------------------------------------------------------" << std::endl;
            	mXycarController->drive(ControllSender::TSTL_LEFT, distance, 0);
            	// std::cout << "TSTL Left!! End!!" << std::endl;
			}

			else if (bbox_container[i][0] == 1)
			{
				std::cout << "TSTL Right!!" << std::endl;
				std::cout << "distance" << distance << std::endl;
				std::cout << "xmin, ymin, xmax, ymax: " << bbox_container[i][1] << " " << bbox_container[i][2] << " " << bbox_container[i][3] << " " << bbox_container[i][4] << std::endl;
				std::cout << "Box Width: " << bbox_container[i][3] - bbox_container[i][1] << std::endl;
				std::cout << "Box Height: " << bbox_container[i][4] - bbox_container[i][2] << std::endl;
				std::cout << "-------------------------------------------------------------------------------" << std::endl;
            	mXycarController->drive(ControllSender::TSTL_RIGHT, distance, 0);
            	// std::cout << "TSTL Right!! End!!" << std::endl;
			}

			else if (bbox_container[i][0] == 2 || bbox_container[i][0] == 3)
			{
				// std::cout << "TSTL Stop!!" << std::endl;
            	mXycarController->drive(ControllSender::TSTL_STOP, distance, 0);
            	// std::cout << "TSTL Stop!! End!!" << std::endl;
			}

			// if traffic light
			else if (bbox_container[i][0] == 4)
			{
        		traffic_xmin = bbox_container[i][1];
        		traffic_ymin = bbox_container[i][2];
        		traffic_xmax = bbox_container[i][3];
        		traffic_ymax = bbox_container[i][4];

				redlight_on = trafficlightDetection(traffic_xmin, traffic_ymin, traffic_xmax, traffic_ymax);
				if (redlight_on)
				{
					std::cout << "TSTL RedLight!!" << std::endl;
            		mXycarController->drive(ControllSender::TSTL_REDLIGHT, distance, 0);
            		std::cout << "TSTL RedLight!! End!!" << std::endl;
				}
				else
				{
					std::cout << "TSTL GreenLight!!" << std::endl;
					mXycarController->drive(ControllSender::TSTL_GREENLIGHT, distance, 0);
					std::cout << "TSTL GreenLight!! End!!" << std::endl;
				}
			}

			// if xycar
			else if (bbox_container[i][0] == 5)
			{
				continue;
			}
		}
	}

    // cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.image[0]));
    // cv::Mat curFrame;
    // cv::cvtColor(src, curFrame, cv::COLOR_RGB2BGR);
    // cv::imshow("detectionCallback", curFrame);
    
	cv::imshow("BEV", mBEV);    
	cv::waitKey(1);
}

template <typename PREC>
bool TSTLDetector<PREC>::trafficlightDetection(int traffic_xmin, int traffic_ymin, int traffic_xmax, int traffic_ymax)
{
    // int traffic_box_height = traffic_ymax - traffic_ymin;

    // cv::Mat traffic_light_redspace = mTSTLFrame(cv::Rect(cv::Point(traffic_xmin, traffic_ymin), cv::Point(traffic_xmax, traffic_ymin + (traffic_box_height / 3))));
    // cv::Mat traffic_light_greenspace = mTSTLFrame(cv::Rect(cv::Point(traffic_xmin, traffic_ymin + (traffic_box_height * 2/3)), cv::Point(traffic_xmax, traffic_ymax)));
    // cv::Mat redspace_hsv;
    // cv::Mat greenspace_hsv;
    // std::vector<cv::Mat> redspace_hsv_split, greenspace_hsv_split;

    // cv::cvtColor(traffic_light_redspace, redspace_hsv, cv::COLOR_BGR2HSV);
    // cv::cvtColor(traffic_light_greenspace, greenspace_hsv, cv::COLOR_BGR2HSV);

    // cv::split(redspace_hsv, redspace_hsv_split);
    // cv::split(greenspace_hsv, greenspace_hsv_split);

	/* traffic light all calculate code
	*/
	cv::Mat traffic_light_space = mTSTLFrame(cv::Rect(cv::Point(traffic_xmin, traffic_ymin), cv::Point(traffic_xmax, traffic_ymax)));
	cv::Mat trafficspace_hsv;
	std::vector<cv::Mat> trafficspace_hsv_split;
	cv::cvtColor(traffic_light_space, trafficspace_hsv, cv::COLOR_BGR2HSV);
	cv::split(trafficspace_hsv, trafficspace_hsv_split);

    // making red_mask, green_mask
    /*
    Threshold values of each color light in the HSV color model
    +-------+-----------+-----------+-----------+
    | Color |     H     |     S     |     V     |
    +-------+-----------+-----------+-----------+
    | Red   | 150 ~ 180 |  80 ~ 255 |  80 ~ 255 |
    | Green |  60 ~ 100 |  75 ~ 255 |  80 ~ 255 |
    +-------+-----------+-----------+-----------+
    */

    cv::Mat red_mask;
    cv::Mat green_mask;

    cv::Scalar lower_red = cv::Scalar(150, 80, 80);
    cv::Scalar upper_red = cv::Scalar(180, 255, 255);
    cv::Scalar lower_green = cv::Scalar(60, 75, 80);
    cv::Scalar upper_green = cv::Scalar(100, 255, 255);

    //cv::inRange(redspace_hsv_split[0], lower_red, upper_red, red_mask);
    //cv::inRange(greenspace_hsv_split[0], lower_green, upper_green, green_mask);

	cv::inRange(trafficspace_hsv_split[0], lower_red, upper_red, red_mask);
	cv::inRange(trafficspace_hsv_split[0], lower_green, upper_green, green_mask);

    int red_pixels = cv::countNonZero(red_mask);
    int green_pixels = cv::countNonZero(green_mask);

    if (red_pixels < green_pixels) 
    {
        return false;
    }
    else
    {
        return true;
    }

}

template <typename PREC>
void TSTLDetector<PREC>::initBEV()
{
	//cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
	mBEV = cv::Mat::zeros(mBEVBlockSize * 6, mBEVBlockSize * 6, CV_8UC3);
	//mBEV(mBEVBlockSize * 6, mBEVBlockSize*, CV_8UC3, cv::Scalar(0, 0, 0))

	for (int y = 1; y <= 5; y++) {
		cv::line(mBEV, cv::Point(0, mBEVBlockSize * y), cv::Point(mBEVBlockSize * 6, mBEVBlockSize * y), cv::Scalar(255, 255, 255), 1);
	}
	for (int x = 1; x <= 5; x++) {
		cv::line(mBEV, cv::Point(mBEVBlockSize * x, 0), cv::Point(mBEVBlockSize * x, mBEVBlockSize * 6), cv::Scalar(255, 255, 255), 1);
	}
}

template <typename PREC>
void TSTLDetector<PREC>::drawObjectRect(std::string clsname, std::vector<cv::Point2f> pts_object)
{
	std::vector<cv::Point2f> dstPoints;
	cv::perspectiveTransform(pts_object, dstPoints, mExtrinsicMat);

	cv::Point2f point = dstPoints[0];
	//std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
	cv::Rect rc_object = cv::Rect(point.x, point.y, 5, 5);
	cv::rectangle(mBEV, rc_object, cv::Scalar(0, 255, 0));
    cv::putText(mBEV, clsname, cv::Point(point.x, point.y-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

}

template <typename PREC>
void TSTLDetector<PREC>::TSTLDetector::findExtrinsicMat()
{
	std::vector<cv::Point2f> pts1, pts2;

	pts1.push_back(cv::Point(198, 285));
	pts1.push_back(cv::Point(260, 285));
	pts1.push_back(cv::Point(322, 285));
	pts1.push_back(cv::Point(384, 285));
	pts1.push_back(cv::Point(446, 285));

	pts1.push_back(cv::Point(156, 294));
	pts1.push_back(cv::Point(239, 294));
	pts1.push_back(cv::Point(322, 294));
	pts1.push_back(cv::Point(405, 294));
	pts1.push_back(cv::Point(488, 294));

	pts1.push_back(cv::Point(76, 310));
	pts1.push_back(cv::Point(199, 310));
	pts1.push_back(cv::Point(322, 310));
	pts1.push_back(cv::Point(446, 310));
	pts1.push_back(cv::Point(568, 310));

	pts1.push_back(cv::Point(-175, 363));
	pts1.push_back(cv::Point(75, 363));
	pts1.push_back(cv::Point(322, 363));
	pts1.push_back(cv::Point(575, 363));
	pts1.push_back(cv::Point(825, 363));

	for (int y = 2; y <= 5; y++) {
		for (int x = 1; x <= 5; x++) {
			pts2.push_back(cv::Point(mBEVBlockSize *x, mBEVBlockSize *y));
		}
	}

	mExtrinsicMat = cv::findHomography(pts1, pts2, cv::RANSAC);

	/*
	[-0.1413860885138055, -1.040802155507865, 316.0058776278547;
	-0.0001117727882957194, -2.075032989161323, 573.9668375470189;
	-1.462738042671476e-06, -0.003846850040132713, 1]
	*/
	return;
}

template class TSTLDetector<float>;
template class TSTLDetector<double>;
} // namespace Xycar