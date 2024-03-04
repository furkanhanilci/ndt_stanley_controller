#include "../include/estimate_pose.h"

PoseEstimator::PoseEstimator()
{
}

void PoseEstimator::updateEstimates(float x, float y)
{

    x_estimates.push_back(x);
    y_estimates.push_back(y);
}

float PoseEstimator::getEstimatedX()
{

    if (!x_estimates.empty())
        return x_estimates.back();
    else
        return 0.0;
}

float PoseEstimator::getEstimatedY()
{

    if (!y_estimates.empty())
        return y_estimates.back();
    else
        return 0.0;
}

