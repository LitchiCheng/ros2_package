#pragma once

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>
#include <libobsensor/hpp/StreamProfile.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include <libobsensor/hpp/Pipeline.hpp>
#include <opencv2/opencv.hpp>

#include <shared_mutex>

class OrbbecCam
{
public:
    OrbbecCam();
    ~OrbbecCam();
    bool wait4Device();
    bool init();
    void run();
    cv::Mat getImg();
    int getImgSizeW();
    int getImgSizeH();
    cv::Mat getDepth();
    int getDepthSizeW();
    int getDepthSizeH();
    bool saveImg();
    bool saveDepth();
private:
    std::shared_ptr<ob::Device> _device{nullptr};
    std::shared_ptr<ob::Pipeline> _pipe{nullptr};
    std::shared_ptr<ob::Config> _config{nullptr};
    ob::PointCloudFilter _point_cloud;
    cv::Mat _intrinsic{(3, 3, CV_32F)};
    cv::Mat _distort{(1, 8, CV_32F)};
    std::shared_mutex _img_lock;
    std::shared_mutex _depth_lock;
    cv::Mat _img{cv::Mat()};
    cv::Mat _depth{cv::Mat()};
    int _rw{480};
    int _rh{640};
};