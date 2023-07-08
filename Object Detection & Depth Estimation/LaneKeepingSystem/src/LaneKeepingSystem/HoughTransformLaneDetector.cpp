// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file HoughTransformLaneDetector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief hough transform lane detector class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <numeric>
#include <cmath>
#include <iomanip>
#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"].as<PREC>();
    mHoughThreshold = config["HOUGH"]["THRESHOLD"].as<int32_t>();
    mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"].as<int32_t>();
    mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::pair<PREC, PREC> HoughTransformLaneDetector<PREC>::getLineParameters(const Lines& lines, Indices& lineIndices, Direction direction)
{
    /*
    PREC raw_x_sum = 0.0f;
	for (int i = 0; i < lineIndices.size(); ++i) {
		Line line = lines[lineIndices[i]];
		PREC x1 = line[0] , x2 = line[2];

		raw_x_sum += (x1 + x2);
	}
    PREC raw_x_avg = raw_x_sum / lineIndices.size();

    Indices tempLineIndices;
    for (int i = 0; i< lineIndices.size(); ++i) {
		Line line = lines[lineIndices[i]];
		PREC x1 = line[0] , x2 = line[2];

        if (direction == Direction::RIGHT){
            if ((x1+x2)/2  <= raw_x_avg) {
                tempLineIndices.push_back(lineIndices[i]);
            }
        }
        else {
            if ((x1+x2)/2  >= raw_x_avg) {
                tempLineIndices.push_back(lineIndices[i]);
            }
        }
    }
    lineIndices = tempLineIndices;
    */

    // TODO : Implement this function
    PREC m = 0.0f;
    PREC b = 0.0f;

	PREC x_sum = 0.0;
	PREC y_sum = 0.0;
	PREC m_sum = 0.0;
	int size = lineIndices.size();

	if (size == 0) {
		return { m, b };
	}

	for (int i = 0; i < lineIndices.size(); ++i) {
		Line line = lines[lineIndices[i]];
		PREC x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

		x_sum += (x1 + x2);
		y_sum += (y1 + y2);
		m_sum += (y2 - y1) / (x2 - x1);
	}

	PREC x_avg = x_sum / (size * 2);
	PREC y_avg = y_sum / (size * 2);
	m = m_sum / size;
	b = y_avg - m * x_avg;

    return { m, b };
}

template <typename PREC>
int32_t HoughTransformLaneDetector<PREC>::getLinePositionX(const Lines& lines, Indices& lineIndices, Direction direction)
{
    // TODO : Implement this function
    int32_t positionX = 0;

    PREC x1, x2;
    std::pair<PREC, PREC> values = getLineParameters(lines, lineIndices, direction);
    PREC m = values.first;
    PREC b = values.second;

    // cannot detect line
    if (m == 0 && b == 0) {
        if (direction == Direction::RIGHT)
            positionX = mImageWidth;
        else
            positionX = 0;
    }
    else {
        positionX = ((mROIHeight / 2) - b) / m;
        b += mROIStartHeight;
        x1 = (mImageHeight - b) / m;
        x2 = ((mImageHeight / 2) - b) / m;

        // draw line
        if (direction == Direction::RIGHT)
            cv::line(mDebugFrame, cv::Point(x1, mImageHeight), cv::Point(x2, (mImageHeight / 2)), kBlue, 3);
        else
            cv::line(mDebugFrame, cv::Point(x1, mImageHeight), cv::Point(x2, (mImageHeight / 2)), kRed, 3);
    }

    return positionX;
}

template <typename PREC>
std::pair<Indices, Indices> HoughTransformLaneDetector<PREC>::divideLines(const Lines& lines)
{
    // TODO : Implement this function
    Indices leftLineIndices;
    Indices rightLineIndices;

	uint8_t low_slope_threshold = 0;
	uint8_t high_slope_threshold = 10;

	for (int32_t i = 0; i < lines.size(); i++) {
		Line line = lines[i];
		PREC x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
		PREC slope;
		if (x2 - x1 == 0) {
			slope = 0;
		}
		else {
			slope = (y2 - y1) / (x2 - x1);
		}
		if ((abs(slope) > low_slope_threshold) && (abs(slope) < high_slope_threshold)) {
			if ((slope < 0) && (x2 < mImageWidth * 0.7)) {
				leftLineIndices.push_back(i);
			}
			else if ((slope > 0) && (x1 > mImageWidth * 0.3)) {
				rightLineIndices.push_back(i);
			}
		}
	}

    return { leftLineIndices, rightLineIndices };
}

template <typename PREC> 
void HoughTransformLaneDetector<PREC>::saveSpeed(PREC my_speed){
    currentSpeed = my_speed;
}

template <typename PREC> 
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
{   
    // for video write
    if (mDebugging)
        image.copyTo(mDebugFrame);

    // convert to gray image
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Gaussain Blur
	cv::Mat blur_image;
	cv::GaussianBlur(gray, blur_image, cv::Size(5,5), 1.5);

    // Canny Edge
    cv::Mat edge_image;
    cv::Canny(blur_image, edge_image, 100, 200);
    // cv::imshow("canny_edge", edge_image);

    // Histogram Stretching
    double global_min, global_max;
    cv::Mat stretch_image;
    cv::minMaxLoc(edge_image, &global_min, &global_max);
    stretch_image = (gray - global_min) * 255 / (global_max - global_min);
	// cv::imshow("stretch_image", stretch_image);

    //// Histogram Equalization
	//Mat equalize_img;
	//equalizeHist(gray, equalize_img);
	//imshow("equalize_img", equalize_img);

    // Binary
    cv::Mat binary_image;
    cv::threshold(stretch_image, binary_image, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // Masking
    // Mask Image load
    cv::Mat mask_image, masked_image;
    mask_image = cv::imread("/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/my_mask.png", cv::IMREAD_GRAYSCALE);
    if (mask_image.empty()) {
        std::cerr << "Mask image load failed!\n";
        exit(1);
    }

    mask_image = ~mask_image;
    cv::bitwise_and(binary_image, mask_image, masked_image);
    cv::dilate(masked_image, masked_image, cv::Mat());
    // cv::imshow("masked_image", masked_image);

    // HoughLinesP
    cv::Mat roi;
    int32_t newRoiHeight = mROIStartHeight - static_cast<int32_t>(currentSpeed);
    if (currentSpeed > 40) {
        roi = edge_image(cv::Rect(0, newRoiHeight, 640, mROIHeight));
        //roi = edge_image(cv::Rect(0, mROIStartHeight - round(currentSpeed), 640, mROIHeight));
        std::cout << "speed : " << std::fixed << std::setprecision(3) << currentSpeed << std::endl;
    }else{
        roi = edge_image(cv::Rect(0, mROIStartHeight, 640, mROIHeight));
    }

    Lines lines;
    cv::HoughLinesP(roi, lines, 1, CV_PI / 180, mHoughThreshold, mHoughMinLineLength, mHoughMaxLineGap);

    // divide left, right lines
    int32_t leftPositionX = 0;
    int32_t rightPositionX = 0;
    std::pair<Indices, Indices> values = divideLines(lines);
    Indices leftLineIndices = values.first;
    Indices rightLineIndices = values.second;
    leftPositionX = getLinePositionX(lines, leftLineIndices, Direction::LEFT);
    rightPositionX = getLinePositionX(lines, rightLineIndices, Direction::RIGHT);

    // draw lines
    drawLines(lines, leftLineIndices, rightLineIndices);

    return { leftPositionX, rightPositionX };
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices)
{
    auto draw_left = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();


            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, cv::Scalar( 255, 0, 0 ), kDebugLineWidth);
        }
    };

    auto draw_right = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();


            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, cv::Scalar(0, 0, 255 ), kDebugLineWidth);
        }
    };


    draw_left(lines, leftLineIndices);
    draw_right(lines, rightLineIndices);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(leftPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(rightPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(estimatedPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(estimatedPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kRed, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(mImageWidth / 2 - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(mImageWidth / 2 + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kBlue, kDebugLineWidth);
}

template class HoughTransformLaneDetector<float>;
template class HoughTransformLaneDetector<double>;
} // namespace Xycar


   