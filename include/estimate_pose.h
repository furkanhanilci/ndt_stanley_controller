#ifndef ESTIMATE_POSE_H
#define ESTIMATE_POSE_H

#include <vector>

class PoseEstimator
{
public:
    PoseEstimator();

    void updateEstimates(float x, float y);
    float getEstimatedX();
    float getEstimatedY();

private:
    std::vector<float> x_estimates;
    std::vector<float> y_estimates;
};

#endif // ESTIMATE_POSE_H
