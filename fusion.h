//

#ifndef FUSION_FUSION_H
#define FUSION_FUSION_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

class FusionDepth{
protected:
    cv::Mat lidar, tof, stereo;
    void ltscompute(bool DayLight);
    void lscompute(bool DayLight);
public:
    cv::Mat lts;
    FusionDepth(cv::Mat &lidar_, cv::Mat &tof_, cv::Mat &stereo_);
    void compute(bool FusionMode = false, bool DayLight = false);
    ~FusionDepth();
private:
};

#endif //FUSION_FUSION_H
