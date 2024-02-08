#ifndef MULTIPLY_ESTIMATES_H
#define MULTIPLY_ESTIMATES_H

#include <vector>

class MultiplyEstimate
{
public:
    MultiplyEstimate();
    void multiplyEstimatesByTwo(float x, float y);
    void multiplyEstimatesByThree(float x, float y);
    float getMultipliedX();
    float getMultipliedY();

private:
    std::vector<float> multiplied_x_estimates;
    std::vector<float> multiplied_y_estimates;
};

#endif
