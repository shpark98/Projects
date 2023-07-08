// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file PIDController.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief PID Controller Class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <cmath>
#include <iostream>
#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
{
    // std::cout << "in" << std::endl;
    if (abs(beforeAngle) > 25) {
        mProportionalGain = 0.25;
        mIntegralGain = 0;
        mDifferentialGain = 0.1;
        // print
        // std::cout << "angle>25 " << std::endl;
    }
    else {
        mProportionalGain = 0.25;
        mIntegralGain = 0;
        mDifferentialGain = 0.1;
        // print
        // std::cout << "straight " << std::endl;
    }

    PREC castError = static_cast<PREC>(errorFromMid);
    mDifferentialGainError = castError - mProportionalGainError;
    mProportionalGainError = castError;
    // if (abs(beforeAngle) < 25) {
    //     std::cout << "ierror reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    //     mIntegralGainError = 0;
    // }
    // else
    mIntegralGainError += castError;
    return mProportionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError + mDifferentialGain * mDifferentialGainError;
}

template <typename PREC>
PREC PIDController<PREC>::saveAngle(PREC angle)
{
    beforeAngle = angle;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar