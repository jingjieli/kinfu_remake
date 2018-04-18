#pragma once

#include <kfusion/kinfu.hpp>
#include <opencv2/core/core.hpp>
#include <string>

#include <OpenNI.h>

namespace kfusion
{
    class KF_EXPORTS OpenNISource
    {
    public:
        typedef kfusion::PixelRGB RGB24;

        enum { PROP_OPENNI_REGISTRATION_ON  = 104 };

        OpenNISource();
        OpenNISource(int device);
        OpenNISource(const std::string& oni_filename, bool repeat = false);

        void open(int device);
        void open(const std::string& oni_filename, bool repeat = false);
        void release();

        ~OpenNISource();

        bool grab(cv::Mat &depth, cv::Mat &image);

        //parameters taken from camera/oni
        int shadow_value, no_sample_value;
        float depth_focal_length_VGA;
        float baseline;               // mm
        double pixelSize;             // mm
        unsigned short max_depth;     // mm

        bool setRegistration (bool value = false);
    private:
        struct Impl;
        cv::Ptr<Impl> impl_;
        void getParams ();

    };

    class KF_EXPORTS OpenNI2Source
    {
    public:
        OpenNI2Source();
        ~OpenNI2Source();

        int open();//int device);
        bool grab(cv::Mat &depth, cv::Mat &image);
    private:
        openni::Device mDevice; 
        openni::VideoStream mColorStream, mDepthStream;
        openni::VideoFrameRef mColorFrame, mDepthFrame;
    };
}
