/// HEADER
#include "filter_lk_tracking.h"

/// PROJECT
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>
#include <QPushButton>

PLUGINLIB_EXPORT_CLASS(cip104::LKTracking, vision_evaluator::BoxedObject)

using namespace cip104;

LKTracking::LKTracking()
    : init_(true), max_count(100), termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), subPixWinSize(10,10), winSize(31,31)
{
}

void LKTracking::filter(cv::Mat& img, cv::Mat& mask)
{
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    if(init_) {
        init(gray, mask);
        init_ = false;

    } else if(!points[0].empty()) {
        std::vector<uchar> status;
        std::vector<float> err;
        if(prevGray.empty())
            gray.copyTo(prevGray);
        cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                             3, termcrit, 0, 0.001);
        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
        {
            if( !status[i] )
                continue;

            points[1][k++] = points[1][i];
            cv::circle(img, points[1][i], 3, cv::Scalar(0,255,0), -1, 8);
        }
        points[1].resize(k);
    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);
}

void LKTracking::update()
{
}

void LKTracking::reset()
{
    init_ = true;
}

void LKTracking::init(cv::Mat& gray, cv::Mat& mask)
{
    cv::goodFeaturesToTrack(gray, points[1], max_count, 0.01, 10, mask, 3, 0, 0.04);
    cv::cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1,-1), termcrit);
}

void LKTracking::insert(QBoxLayout* layout)
{
    QPushButton* reset = new QPushButton("reset");
    layout->addWidget(reset);

    connect(reset, SIGNAL(clicked()), this, SLOT(reset()));
}
