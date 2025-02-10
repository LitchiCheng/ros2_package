#include "orbbec.h"

#include <thread>
#include <log.hpp>
#include <lock.hpp>

OrbbecCam::OrbbecCam() {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_FATAL);
}

OrbbecCam::~OrbbecCam() {
    if(_pipe){
        _pipe->stop();
    }
    _pipe   = nullptr;
    _config = nullptr;
    _device = nullptr;
}

bool OrbbecCam::wait4Device()
{
    ob::Context ctx;
    auto devList = ctx.queryDeviceList();
    int camNum   = devList->deviceCount();
    std::stringstream ss;
    ss << "discover camera number is " << camNum;
    CGU::LOGDD(ss.str());
    for(int i = 0; i < camNum; i++){
        if(!_device){
            _device = devList->getDevice(i);
        }
        auto devInfo = _device->getDeviceInfo();
        auto serial_number = devInfo->serialNumber();
        break;
    }

    if(!_device){
        return false;
    }
    return true;
}

bool OrbbecCam::init()
{
    _pipe = std::make_shared<ob::Pipeline>(_device);
    _config = std::make_shared<ob::Config>();
    try{
        //配置rgb摄像头的参数
        auto colorProfiles = _pipe->getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
        try{
            //640，480，RGB，30帧
            colorProfile = colorProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_RGB, 30);
        } catch(ob::Error &e){
            colorProfile = colorProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_UNKNOWN, 30);
        }
        int rw = colorProfile->width();
        int rh = colorProfile->height();
        std::stringstream ss;
        ss << "rgb size: " << rh << " * " << rw;
        CGU::LOGDD(ss.str());
        _config->enableStream(colorProfile);
    } catch(ob::Error &e) {
        std::stringstream ss;
        ss << "Current device is not support color sensor!";
        CGU::LOGDE(ss.str());
    }
    try {
        auto depthProfiles = _pipe->getStreamProfileList(OB_SENSOR_DEPTH);
        std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
        try {
            depthProfile = depthProfiles->getVideoStreamProfile(640, 480,OB_FORMAT_Y11, 30);
        } catch(ob::Error &e){
            depthProfile = depthProfiles->getVideoStreamProfile(640, 480, OB_FORMAT_UNKNOWN, 30);
        }

        int pw = depthProfile->width();
        int ph = depthProfile->height();
        std::stringstream ss;
        ss << "depth size : " << ph << " * " << pw;
        CGU::LOGDD(ss.str());
        _config->enableStream(depthProfile);
    } catch(ob::Error &e) {
        std::stringstream ss;
        ss << "Current device is not support depth sensor!";
        CGU::LOGDE(ss.str());
    }
    if(_device->isPropertySupported(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL,OB_PERMISSION_READ)){
        _config->setAlignMode(ALIGN_D2C_HW_MODE);
    } else{
        _config->setAlignMode(ALIGN_D2C_SW_MODE);
    }

    if(!_pipe)
        return 0;
    _pipe->start(_config);
    
    auto cameraParam = _pipe->getCameraParam();
    _point_cloud.setCameraParam(cameraParam);
    _intrinsic.at<float>(0, 0) = cameraParam.rgbIntrinsic.fx;
    _intrinsic.at<float>(0, 1) = 0;
    _intrinsic.at<float>(0, 2) = cameraParam.rgbIntrinsic.cx;
    _intrinsic.at<float>(1, 0) = 0;
    _intrinsic.at<float>(1, 1) = cameraParam.rgbIntrinsic.fy;
    _intrinsic.at<float>(1, 2) = cameraParam.rgbIntrinsic.cy;
    _intrinsic.at<float>(2, 0) = 0;
    _intrinsic.at<float>(2, 1) = 0;
    _intrinsic.at<float>(2, 2) = 1;
    _distort.at<float>(0,0)   =  cameraParam.rgbDistortion.k1;
    _distort.at<float>(0,1)   =  cameraParam.rgbDistortion.k2;
    _distort.at<float>(0,2)   =  cameraParam.rgbDistortion.p1;
    _distort.at<float>(0,3)   =  cameraParam.rgbDistortion.p2;
    _distort.at<float>(0,4)   =  cameraParam.rgbDistortion.k3;
    _distort.at<float>(0,5)   =  cameraParam.rgbDistortion.k4;
    _distort.at<float>(0,6)   =  cameraParam.rgbDistortion.k5;
    _distort.at<float>(0,7)   =  cameraParam.rgbDistortion.k6;
    return false;
}

void OrbbecCam::run() 
{
    while(true)
    {
        auto frameSet = _pipe->waitForFrames(1000);
     
        if(frameSet == nullptr){
            std::stringstream ss;
            ss << "get frame fail!";
            CGU::LOGDE(ss.str());
            continue;
        }
        if(frameSet != nullptr && frameSet->depthFrame() != nullptr && frameSet->colorFrame() != nullptr) {
            auto depthValueScale = frameSet->depthFrame()->getValueScale();
            break;
        };
        if(!frameSet || !frameSet->depthFrame() || !frameSet->colorFrame()){
            std::stringstream ss;
            ss << "frameSet or depthFrame or colorFrame is null";
            CGU::LOGDE(ss.str());
        }
      
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    while(true)
    {
        auto frameSet = _pipe->waitForFrames(100);
        if(frameSet == nullptr){
            continue;
        }
        if(frameSet != nullptr && frameSet->depthFrame() != nullptr && frameSet->colorFrame() != nullptr){
            auto depthValueScale = frameSet->depthFrame()->getValueScale();
            _point_cloud.setPositionDataScaled(depthValueScale);
            try{
                int rw = frameSet->colorFrame()->width();
                int rh = frameSet->colorFrame()->height();
                {
                    CGU::WRITE_LOCK(_img_lock);
                    _img = cv::Mat(rh, rw, CV_8UC3, frameSet->colorFrame()->data());
                }
                {
                    CGU::WRITE_LOCK(_depth_lock);
                    _depth = cv::Mat(frameSet->depthFrame()->height(), frameSet->depthFrame()->width(), CV_16UC1, frameSet->depthFrame()->data());
                }
                _point_cloud.setCreatePointFormat(OB_FORMAT_POINT);
                std::shared_ptr<ob::Frame> frame = _point_cloud.process(frameSet);
                // calc frame freq
                auto now = std::chrono::high_resolution_clock::now();
                auto freq = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
                start = now;
                std::stringstream ss;
                ss << "frame freq is " << freq;
                // CGU::LOGDI(ss.str());
            } catch(std::exception &e){
                std::stringstream ss;
                ss << "handle frame err " << e.what();
                CGU::LOGDE(ss.str());
            }
        } else {
            std::stringstream ss;
            ss << "get color frame or depth frame failed!";
            CGU::LOGDE(ss.str());
        }
    }
}

cv::Mat OrbbecCam::getImg()
{
    CGU::READ_LOCK(_img_lock);
    return _img;
}

int OrbbecCam::getImgSizeW()
{
    return _rw;
}

int OrbbecCam::getImgSizeH()
{
    return _rh;
}

cv::Mat OrbbecCam::getDepth()
{
    CGU::READ_LOCK(_depth_lock);
    return _depth;
}

int OrbbecCam::getDepthSizeW()
{
    return _rw;
}

int OrbbecCam::getDepthSizeH()
{
    return _rh;
}

bool OrbbecCam::saveImg()
{
    cv::Mat img = getImg();
    if(!img.empty())
        cv::imwrite("img.jpg", img);
    return true;
}

bool OrbbecCam::saveDepth()
{
    cv::Mat depth = getDepth();
    if(!depth.empty())
        cv::imwrite("depth.jpg", depth);
    return true;
}
