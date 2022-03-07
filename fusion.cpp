//
#include "fusion.h"
typedef unsigned char uint8;
using namespace cv;

FusionDepth::FusionDepth(Mat &lidar_, Mat &tof_, Mat &stereo_) : lidar(lidar_), tof(tof_), stereo(stereo_){
    lidar.convertTo(lidar, CV_32F);
    tof.convertTo(tof, CV_32F);
    stereo.convertTo(stereo, CV_32F);
    tof.setTo(0, tof>5000);
    tof = tof / 10000.0;
    //lidar.setTo(0, lidar>10000);
    lidar = lidar / 10000.0;
    stereo.setTo(0, stereo>8000);
    stereo = stereo / 10000.0;
}

void FusionDepth::ltscompute(bool DayLight) {
    //choose the fusion mode
    if(DayLight){
    }

    Mat lidar_;
    lidar_ = lidar.clone();
    Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(lidar_, lidar, kernel);

    Mat tof_;
    tof.copyTo(tof_);
    Mat stereo_;
    stereo.copyTo(stereo_);

    Mat mask_tof = (tof_!=0) / 255;
    mask_tof.convertTo(mask_tof, CV_8UC1);
    tof = tof_ + stereo_.setTo(0, mask_tof);

    stereo.copyTo(stereo_);
    Mat mask_stereo_ = (stereo_!=0) / 255;
    mask_stereo_.convertTo(mask_stereo_, CV_8UC1);
    stereo = stereo_ + tof_.setTo(0, mask_stereo_);

    //get mask of gradient in stereo
    Mat mask_stereo = (stereo!=0) / 255;
    mask_stereo.convertTo(mask_stereo, CV_8UC1);
    Mat mask_h = mask_stereo(Range::all(), Range(1, stereo.cols));
    Mat add = Mat::ones(stereo.rows, 1, CV_8UC1);
    hconcat(mask_h, add, mask_h);
    Mat mask_v = mask_stereo(Range(1, stereo.rows), Range::all());
    add = Mat::ones(1, stereo.cols, CV_8UC1);
    vconcat(mask_v, add, mask_v);
    Mat mask_g_stereo = mask_stereo.mul(mask_h).mul(mask_v);

    //set mask, padding and pointer
    Mat lidar_padding = lidar.clone();
    Mat l_padding = Mat::zeros(lidar.rows+2, lidar.cols+2, CV_32F);
    copyMakeBorder(lidar_padding, lidar_padding, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    Mat mask = Mat::zeros(lidar.rows, lidar.cols, CV_8UC1);
    copyMakeBorder(mask, mask, 1, 1, 1, 1, BORDER_CONSTANT, 1);
    copyMakeBorder(tof, tof, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    copyMakeBorder(stereo, stereo, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    copyMakeBorder(mask_g_stereo, mask_g_stereo, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    int y = lidar_padding.cols;
    float * t = tof.ptr<float>(0);
    float * s = stereo.ptr<float>(0);
    float * lp = lidar_padding.ptr<float>(0);
    uint8 * g = mask_g_stereo.ptr<uint8>(0);

    //get started
    for(uint8 num=0; num < 10; num++){
        float * l = l_padding.ptr<float>(0);
        for(int i=0; i<lidar_padding.rows * lidar_padding.cols; i++){
            if(lp[i]!=0){
                if(t[i]!=0 && (lp[i] - t[i])>0.01 ){
                    l[i] = t[i];
                    lp[i]=0;
                }
                if((lp[i+1]==0) && (g[i]==1) && (l[i+1]==0)){
                    l[i+1] = lp[i] + (s[i+1] - s[i]) * lp[i] / s[i];
                    if(t[i+1]!=0 && (l[i+1] - t[i+1])>0.01 ){
                        l[i+1] = t[i+1];
                        lp[i]=0;
                    }}
                if((lp[i+y]==0) && (g[i]==1) && (l[i+y]==0)) {
                    l[i + y] = lp[i] + (s[i + y] - s[i]) * lp[i] / s[i];
                    if(t[i+y]!=0 && (l[i+y] - t[i+y])>0.01 ){
                        l[i+y] = t[i+y];
                        lp[i]=0;
                    }
                }
                if((lp[i-1]==0) && (g[i-1]==1) && (l[i-1]==0)) {
                    l[i - 1] = lp[i] - (s[i] - s[i - 1]) * lp[i] / s[i];
                    if(t[i-1]!=0 && (l[i-1] - t[i-1])>0.01 ){
                        l[i-1] = t[i-1];
                        lp[i]=0;
                    }
                }
                if((lp[i-y]==0) && (g[i-y]==1) && (l[i-y]==0)) {
                    l[i - y] = lp[i] - (s[i] - s[i - y]) * lp[i] / s[i];
                    if(t[i-y]!=0 && (l[i-y] - t[i-y])>0.01 ){
                        l[i-y] = t[i-y];
                        lp[i]=0;
                    }
                }
            }
        }
        lidar_padding = lidar_padding + l_padding;
        l_padding = Mat::zeros(lidar.rows+2, lidar.cols+2, CV_32F);
        lidar_padding.setTo(0, mask);
    }
    tof = tof(Range(1, lidar.rows+1), Range(1, lidar.cols+1));
    stereo = stereo(Range(1, lidar.rows+1), Range(1, lidar.cols+1));
    lidar_padding = lidar_padding(Range(1, lidar.rows+1), Range(1, lidar.cols+1));
    Mat flag = (lidar_padding==0) / 255;
    flag.convertTo(flag, CV_32F);
    //lts = lidar;

    lts = lidar_padding + tof.mul(flag);
    //lts = lidar;
}

void FusionDepth::lscompute(bool DayLight){
    if(DayLight){
        Mat tof_stereo = tof.clone();
        tof = stereo.clone();
        stereo = tof_stereo.clone();
    }

    //get mask of gradient in stereo
    Mat mask_stereo = (stereo!=0) / 255;
    mask_stereo.convertTo(mask_stereo, CV_8UC1);
    Mat mask_h = mask_stereo(Range::all(), Range(1, stereo.cols));
    Mat add = Mat::ones(stereo.rows, 1, CV_8UC1);
    hconcat(mask_h, add, mask_h);
    Mat mask_v = mask_stereo(Range(1, stereo.rows), Range::all());
    add = Mat::ones(1, stereo.cols, CV_8UC1);
    vconcat(mask_v, add, mask_v);
    Mat mask_g_stereo = mask_stereo.mul(mask_h).mul(mask_v);

    //get lidar padding and mask of lidar padding
    Mat mask_lidar_padding = (lidar!=0) / 255, lidar_padding = lidar.clone();
    mask_lidar_padding.convertTo(mask_lidar_padding, CV_8UC1);
    copyMakeBorder(mask_lidar_padding, mask_lidar_padding, 10, 10, 10, 10, BORDER_CONSTANT, 0);
    copyMakeBorder(lidar_padding, lidar_padding, 10, 10, 10, 10, BORDER_CONSTANT, 0);

    //get stereo padding and mask of stereo padding
    Mat mask_stereo_padding = mask_stereo.clone(), stereo_padding = stereo.clone();
    copyMakeBorder(mask_stereo_padding, mask_stereo_padding, 10, 10, 10, 10, BORDER_CONSTANT, 0);
    copyMakeBorder(stereo_padding, stereo_padding, 10, 10, 10, 10, BORDER_CONSTANT, 0);

    //get the _tof
    std::vector<cv::Point2i> locations;
    findNonZero(mask_lidar_padding, locations);
    mask_lidar_padding.convertTo(mask_lidar_padding, CV_32F);
    mask_stereo_padding.convertTo(mask_stereo_padding, CV_32F);
    Mat _tof = mask_stereo_padding.clone();
    Mat tof_ = mask_stereo_padding.clone();
    Mat mask_fusion = Mat::zeros(stereo_padding.rows, stereo_padding.cols, CV_8UC1);
    int x, y;
    for (int i=0; i<locations.size(); i++) {
        x = locations[i].y;
        y = locations[i].x;
        mask_fusion(Range(x - 10, x + 9), Range(y - 10, y + 9)) = 1;
        Scalar ans1 = sum(stereo_padding(Range(x - 10, x + 9), Range(y - 10, y + 9)));
        Scalar ans2 = sum(mask_stereo_padding(Range(x - 10, x + 9), Range(y - 10, y + 9)));
        Scalar ans3 = sum(lidar_padding(Range(x - 10, x + 9), Range(y - 10, y + 9)));
        Scalar ans4 = sum(mask_lidar_padding(Range(x - 10, x + 9), Range(y - 10, y + 9)));
        float ans1_ = ans1.val[0], ans2_ = ans2.val[0], ans3_ = ans3.val[0], ans4_ = ans4.val[0];
        if (ans2_ != 0) {
            float ans = ans1_ / ans2_ - ans3_ / ans4_;
            if (abs(ans) > 0.025)
                _tof(Range(x - 10, x + 9), Range(y - 10, y + 9)) = 0;
        }
    }
    _tof = _tof(Range(10, tof.rows+10), Range(10, tof.cols+10));
    _tof.convertTo(_tof, CV_8UC1);
    mask_fusion = mask_fusion(Range(10, tof.rows+10), Range(10, tof.cols+10));

    //set mask, padding and pointer
    lidar_padding = lidar_padding(Range(10, lidar.rows+10), Range(10, lidar.cols+10));
    Mat l_padding = Mat::zeros(lidar.rows+2, lidar.cols+2, CV_32F);
    copyMakeBorder(lidar_padding, lidar_padding, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    Mat mask = Mat::zeros(lidar.rows, lidar.cols, CV_8UC1);
    copyMakeBorder(mask, mask, 1, 1, 1, 1, BORDER_CONSTANT, 1);
    copyMakeBorder(_tof, _tof, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    copyMakeBorder(stereo, stereo, 1, 1, 1, 1, BORDER_CONSTANT, 0);
    copyMakeBorder(mask_g_stereo, mask_g_stereo, 1, 1, 1, 1, BORDER_CONSTANT, 0);

    y = lidar_padding.cols;
    float * lp = lidar_padding.ptr<float>(0);
    float * s = stereo.ptr<float>(0);
    uint8 * g = mask_g_stereo.ptr<uint8>(0);
    uint8 * t = _tof.ptr<uint8>(0);

    //get started
    for(uint8 num=0; num < 10; num++){
        float * l = l_padding.ptr<float>(0);
        for(int i=0; i<lidar_padding.rows * lidar_padding.cols; i++){
            if(lp[i]!=0){
                if((lp[i+1]==0) && (g[i]==1) && (l[i+1]==0)){
                    l[i+1] = lp[i] + (s[i+1] - s[i]) * lp[i] / s[i];
                    if(s[i+1]!=0 && (l[i+1] - s[i+1])>0.01 ){
                        l[i+1] = s[i+1];
                        lp[i]=0;
                    }}
                if((lp[i+y]==0) && (g[i]==1) && (l[i+y]==0)) {
                    l[i + y] = lp[i] + (s[i + y] - s[i]) * lp[i] / s[i];
                    if(s[i+y]!=0 && (l[i+y] - s[i+y])>0.01 ){
                        l[i+y] = s[i+y];
                        lp[i]=0;
                    }
                }
                if((lp[i-1]==0) && (g[i-1]==1) && (l[i-1]==0)) {
                    l[i - 1] = lp[i] - (s[i] - s[i - 1]) * lp[i] / s[i];
                    if(s[i-1]!=0 && (l[i-1] - s[i-1])>0.01 ){
                        l[i-1] = s[i-1];
                        lp[i]=0;
                    }
                }
                if((lp[i-y]==0) && (g[i-y]==1) && (l[i-y]==0)) {
                    l[i - y] = lp[i] - (s[i] - s[i - y]) * lp[i] / s[i];
                    if(s[i-y]!=0 && (l[i-y] - s[i-y])>0.01 ){
                        l[i-y] = s[i-y];
                        lp[i]=0;
                    }
                }
            }
        }
        lidar_padding = lidar_padding + l_padding;
        l_padding = Mat::zeros(lidar.rows+2, lidar.cols+2, CV_32F);
        lidar_padding.setTo(0, mask);
    }

    stereo = stereo(Range(1, lidar.rows+1), Range(1, lidar.cols+1));
    lidar_padding = lidar_padding(Range(1, lidar.rows+1), Range(1, lidar.cols+1));
    mask_fusion = (lidar_padding!=0) / 255;
    mask_fusion.convertTo(mask_fusion, CV_8UC1);
    lts = lidar_padding + stereo.setTo(0, mask_fusion);
}

void FusionDepth::compute(bool FusionMode, bool DayLight){
    if(FusionMode)
        lscompute(DayLight);
    else
        ltscompute(DayLight);
}

FusionDepth::~FusionDepth(){
}