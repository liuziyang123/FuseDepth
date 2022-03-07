#include <ctime>
#include "fusion.h"
typedef unsigned char uint8;

int main(int argc, char* argv[]) {
    clock_t startTime, endTime;

    //read depth
    std::string dir = "../";
    cv::Mat lidar = cv::imread(((std::string)dir).append(argv[1]).append("/lidar.png"), cv::IMREAD_ANYDEPTH);
    cv::Mat tof = cv::imread(((std::string)dir).append(argv[1]).append("/tof.png"), cv::IMREAD_ANYDEPTH);
    cv::Mat stereo = cv::imread(((std::string)dir).append(argv[1]).append("/stereo.png"), cv::IMREAD_ANYDEPTH);

    //fusionmode: if false, fuse lidar, tof, and stereo; if true, fuse lidar with tof or stereo.
    //daylight: if false fuse lidar with stereo; if true, fuse liadr with tof.
    bool fusionmode = false, daylight = false;

    //fusion
    FusionDepth fd(lidar, tof, stereo);
    startTime = clock();
    fd.compute(fusionmode, daylight);
    cv::Mat output = fd.lts;
    endTime = clock();

    std::cout<<output.rows<<" "<<output.cols<<"\n";
    std::cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", output);
    output = output * 255;
    cv::imwrite("output.png", output);
    cv::waitKey(0);
    return 0;
}
